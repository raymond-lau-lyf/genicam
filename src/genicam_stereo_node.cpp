/**
 * @file genicam_stereo_node.cpp
 * @brief 双目相机同步采集节点
 * 
 * 解决问题：两个硬件同步触发的相机，由于网络延迟不同导致时间戳不一致
 * 方案：在同一进程中管理两个相机，配对后统一时间戳发布
 * 
 * 防错位机制：
 * - 帧间隔 40ms (25Hz)，最大配对时间差设为 20ms (帧间隔的一半)
 * - 如果两帧时间差 > 20ms，则不是同一对，丢弃较早的帧
 * - 这样数学上保证不会发生 left_i 与 right_i+1 错位配对
 */

#include <ros/ros.h>
#include <arv.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <chrono>
#include <image_transport/image_transport.h>
#include <signal.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <deque>
#include <std_msgs/Time.h>

const uint64_t CONVERT_NANO = 1000000000L;
const uint64_t CONVERT_MICRO = 1000000L;

// 默认配置常量
const int DEFAULT_RETRY_COUNT = 5;
const int DEFAULT_RETRY_DELAY_MS = 500;
const int CLEANUP_DELAY_MS = 300;
const int DISCOVERY_DELAY_MS = 200;

// 帧同步配置
const uint64_t DEFAULT_FRAME_INTERVAL_NS = 40 * CONVERT_MICRO;  // 40ms = 25Hz
const uint64_t DEFAULT_MAX_PAIR_DELAY_NS = 25 * CONVERT_MICRO;  // 25ms，给网络抖动留余量

std::set<std::string> supported_format = {"YUV422_8", "BayerRG8", "Mono8", "Mono16"};

// 全局清理状态标志
static std::atomic<bool> g_cleanup_done(false);
static std::mutex g_cleanup_mutex;

// PTP 触发时间戳支持 - 队列机制
static const int TRIGGER_QUEUE_SIZE = 8;  // 保存最近 N 个触发时间
static std::atomic<bool> g_use_ptp_trigger(false);
static std::mutex g_trigger_time_mutex;
static std::deque<ros::Time> g_trigger_time_queue;  // 触发时间队列
static uint64_t g_trigger_receive_count = 0;

void trigger_time_callback(const std_msgs::Time::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_trigger_time_mutex);
    
    // 添加到队列
    g_trigger_time_queue.push_back(msg->data);
    
    // 保持队列大小
    while (g_trigger_time_queue.size() > TRIGGER_QUEUE_SIZE) {
        g_trigger_time_queue.pop_front();
    }
    
    g_trigger_receive_count++;
    
    // 每50次打印一次状态
    if (g_trigger_receive_count % 50 == 0) {
        ros::Time now = ros::Time::now();
        double latency_ms = (now - msg->data).toSec() * 1000.0;
        ROS_INFO("TriggerSync: received %lu triggers, queue_size=%zu, PTP=%u.%09u, latency=%.2fms",
                 g_trigger_receive_count, g_trigger_time_queue.size(), 
                 msg->data.sec, msg->data.nsec, latency_ms);
    }
}

// 根据系统时间戳从队列中找到最匹配的 PTP 触发时间
ros::Time find_best_trigger_time(uint64_t sys_timestamp_ns) {
    std::lock_guard<std::mutex> lock(g_trigger_time_mutex);
    
    if (g_trigger_time_queue.empty()) {
        return ros::Time(0);  // 无效
    }
    
    ros::Time sys_time;
    sys_time.fromNSec(sys_timestamp_ns);
    
    // 找到与 sys_time 最接近且在其之前的触发时间
    // （因为触发时间应该在图像到达之前）
    ros::Time best_trigger(0);
    double min_diff = 1e9;  // 很大的值
    
    for (const auto& trigger_time : g_trigger_time_queue) {
        // 触发时间应该在系统时间之前（图像传输需要时间）
        double diff_ms = (sys_time - trigger_time).toSec() * 1000.0;
        
        // 合理范围：0ms < diff < 60ms（触发到图像到达的延迟）
        if (diff_ms > 0 && diff_ms < 60.0 && diff_ms < min_diff) {
            min_diff = diff_ms;
            best_trigger = trigger_time;
        }
    }
    
    return best_trigger;
}

// 立体相机参数结构
struct stereo_cam_params {
    // 左右相机 IP
    std::string camera_ip_left;
    std::string camera_ip_right;
    
    // 发布话题
    std::string publish_topic_left;
    std::string publish_topic_right;
    
    // 同步配置
    std::string sync_strategy;      // "first", "average", "left", "right"
    int sync_timeout_ms;            // 帧配对超时(ms)
    
    // 共用相机参数
    std::string pixel_format;
    std::string palette;
    int binning_h, binning_w;
    std::string exposure_auto;
    double exposure_time;
    std::string gain_auto;
    double gain;
    bool vis_img;
    bool externalsync;
    bool triggermode;
    int gev_packet_size, gev_packet_delay;
    int buffer_num;
    int retry_count;
    int retry_delay_ms;
    int auto_cal_time;  // 自动校准时间 (AutoCalTime)
} params;

// 待配对帧结构
struct PendingFrame {
    cv::Mat image;
    uint64_t system_timestamp;
    bool valid;
    
    PendingFrame() : valid(false), system_timestamp(0) {}
    void reset() { valid = false; image.release(); system_timestamp = 0; }
};

// 立体帧同步管理器
class StereoFrameSync {
public:
    StereoFrameSync() : max_pair_delay_ns_(DEFAULT_MAX_PAIR_DELAY_NS) {}
    
    void set_max_pair_delay(uint64_t delay_ns) { max_pair_delay_ns_ = delay_ns; }
    void set_publishers(image_transport::Publisher* left_pub, image_transport::Publisher* right_pub) {
        left_pub_ = left_pub;
        right_pub_ = right_pub;
    }
    void set_sync_strategy(const std::string& strategy) { sync_strategy_ = strategy; }
    
    // 左帧到达
    void on_left_frame(const cv::Mat& image, uint64_t sys_timestamp) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        
        if (pending_right_.valid) {
            // 右帧已在等待，尝试配对
            int64_t time_diff = std::abs((int64_t)(sys_timestamp - pending_right_.system_timestamp));
            
            if ((uint64_t)time_diff < max_pair_delay_ns_) {
                // 配对成功
                uint64_t unified_ts = compute_unified_timestamp(sys_timestamp, pending_right_.system_timestamp);
                publish_stereo_pair(image, pending_right_.image, unified_ts);
                pending_right_.reset();
                pair_success_count_++;
            } else {
                // 时间差太大，判断哪个更早
                if (pending_right_.system_timestamp < sys_timestamp) {
                    // 右帧太旧，丢弃右帧，保留左帧
                    ROS_WARN("Dropping old right frame (time_diff=%.2fms > %.2fms)", 
                             time_diff / 1e6, max_pair_delay_ns_ / 1e6);
                    pending_right_.reset();
                    pending_left_.image = image.clone();
                    pending_left_.system_timestamp = sys_timestamp;
                    pending_left_.valid = true;
                    drop_count_++;
                } else {
                    // 左帧太旧（不应该发生），丢弃左帧
                    ROS_WARN("Dropping old left frame (time_diff=%.2fms)", time_diff / 1e6);
                    drop_count_++;
                }
            }
        } else {
            // 没有待配对的右帧，缓存左帧
            pending_left_.image = image.clone();
            pending_left_.system_timestamp = sys_timestamp;
            pending_left_.valid = true;
        }
    }
    
    // 右帧到达
    void on_right_frame(const cv::Mat& image, uint64_t sys_timestamp) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        
        if (pending_left_.valid) {
            // 左帧已在等待，尝试配对
            int64_t time_diff = std::abs((int64_t)(sys_timestamp - pending_left_.system_timestamp));
            
            if ((uint64_t)time_diff < max_pair_delay_ns_) {
                // 配对成功
                uint64_t unified_ts = compute_unified_timestamp(pending_left_.system_timestamp, sys_timestamp);
                publish_stereo_pair(pending_left_.image, image, unified_ts);
                pending_left_.reset();
                pair_success_count_++;
            } else {
                // 时间差太大，判断哪个更早
                if (pending_left_.system_timestamp < sys_timestamp) {
                    // 左帧太旧，丢弃左帧，保留右帧
                    ROS_WARN("Dropping old left frame (time_diff=%.2fms > %.2fms)", 
                             time_diff / 1e6, max_pair_delay_ns_ / 1e6);
                    pending_left_.reset();
                    pending_right_.image = image.clone();
                    pending_right_.system_timestamp = sys_timestamp;
                    pending_right_.valid = true;
                    drop_count_++;
                } else {
                    // 右帧太旧（不应该发生），丢弃右帧
                    ROS_WARN("Dropping old right frame (time_diff=%.2fms)", time_diff / 1e6);
                    drop_count_++;
                }
            }
        } else {
            // 没有待配对的左帧，缓存右帧
            pending_right_.image = image.clone();
            pending_right_.system_timestamp = sys_timestamp;
            pending_right_.valid = true;
        }
    }
    
    void print_stats() {
        ROS_INFO("Stereo sync stats: %zu pairs published, %zu frames dropped", 
                 pair_success_count_, drop_count_);
    }
    
private:
    uint64_t compute_unified_timestamp(uint64_t left_ts, uint64_t right_ts) {
        if (sync_strategy_ == "first") {
            return std::min(left_ts, right_ts);
        } else if (sync_strategy_ == "average") {
            return (left_ts + right_ts) / 2;
        } else if (sync_strategy_ == "left") {
            return left_ts;
        } else if (sync_strategy_ == "right") {
            return right_ts;
        }
        return std::min(left_ts, right_ts);  // 默认 first
    }
    
    void publish_stereo_pair(const cv::Mat& left_img, const cv::Mat& right_img, uint64_t unified_ts) {
        ros::Time stamp;
        ros::Time sys_stamp = ros::Time().fromNSec(unified_ts);
        
        // 判断是否使用 PTP 触发时间戳
        if (g_use_ptp_trigger.load()) {
            // 使用队列机制查找最佳匹配的触发时间
            ros::Time best_trigger = find_best_trigger_time(unified_ts);
            
            if (best_trigger.toSec() > 0) {
                stamp = best_trigger;
                // 计算系统时间与 PTP 时间的差值，用于调试
                double diff_ms = (sys_stamp - stamp).toSec() * 1000.0;
                if (pair_success_count_ % 25 == 0) {
                    ROS_INFO("StereoSync: using PTP trigger time, sys-ptp_diff=%.2fms (transmission delay)",
                             diff_ms);
                }
            } else {
                // PTP 时间戳不可用，回退到系统时间戳
                stamp = sys_stamp;
                ROS_WARN_THROTTLE(1.0, "StereoSync: no matching PTP trigger found, using system time");
            }
        } else {
            stamp = sys_stamp;
        }
        
        // 发布左图
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", left_img).toImageMsg();
        left_msg->header.stamp = stamp;
        left_msg->header.frame_id = "stereo_left";
        left_pub_->publish(left_msg);
        
        // 发布右图
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", right_img).toImageMsg();
        right_msg->header.stamp = stamp;
        right_msg->header.frame_id = "stereo_right";
        right_pub_->publish(right_msg);
        
        // 记录用于统计
        if (last_publish_ts_ > 0) {
            double interval_ms = (unified_ts - last_publish_ts_) / 1e6;
            // 每隔一段时间打印一次帧间隔统计
            interval_sum_ms_ += interval_ms;
            interval_count_++;
            if (interval_count_ >= 25) {  // 约1秒统计一次
                double avg_interval = interval_sum_ms_ / interval_count_;
                ROS_INFO("Stereo publish: avg_interval=%.4fms (target=40ms), fps=%.4f", 
                         avg_interval, 1000.0 / avg_interval);
                interval_sum_ms_ = 0;
                interval_count_ = 0;
            }
        }
        last_publish_ts_ = unified_ts;
    }
    
    std::mutex sync_mutex_;
    PendingFrame pending_left_;
    PendingFrame pending_right_;
    
    uint64_t max_pair_delay_ns_;
    std::string sync_strategy_ = "first";
    
    image_transport::Publisher* left_pub_ = nullptr;
    image_transport::Publisher* right_pub_ = nullptr;
    
    // 统计
    size_t pair_success_count_ = 0;
    size_t drop_count_ = 0;
    uint64_t last_publish_ts_ = 0;
    double interval_sum_ms_ = 0;
    size_t interval_count_ = 0;
};

// 全局帧同步管理器
StereoFrameSync g_stereo_sync;

// 单个相机的 stream callback 数据
struct CameraStreamData {
    ArvStream* stream = nullptr;
    ArvCamera* camera = nullptr;
    size_t counter = 0;
    size_t payload = 0;
    std::string pixel_format;
    std::string side;  // "left" or "right"
    gboolean done = FALSE;
};

// 全局相机对象
CameraStreamData g_left_camera;
CameraStreamData g_right_camera;

// 处理图像 buffer，返回 RGB 图像
cv::Mat process_buffer_to_rgb(ArvBuffer* buffer, const std::string& pixel_format) {
    cv::Mat img_rgb;
    
    if (supported_format.find(pixel_format) == supported_format.end()) {
        ROS_ERROR("Unsupported pixel format: %s", pixel_format.c_str());
        return img_rgb;
    }
    
    int img_w = arv_buffer_get_image_width(buffer);
    int img_h = arv_buffer_get_image_height(buffer);
    size_t data_size;
    const void* data = arv_buffer_get_image_data(buffer, &data_size);
    
    if (pixel_format == "YUV422_8") {
        cv::Mat data_wrapper(cv::Size(img_w, img_h), CV_8UC2, (void*)data);
        cv::cvtColor(data_wrapper, img_rgb, cv::COLOR_YUV2RGB_YUY2);
    } else if (pixel_format == "BayerRG8") {
        cv::Mat data_wrapper(cv::Size(img_w, img_h), CV_8U, (void*)data);
        cv::cvtColor(data_wrapper, img_rgb, cv::COLOR_BayerRG2BGR);
    } else if (pixel_format == "Mono8") {
        cv::Mat data_wrapper(cv::Size(img_w, img_h), CV_8U, (void*)data);
        cv::cvtColor(data_wrapper, img_rgb, cv::COLOR_GRAY2RGB);
    } else if (pixel_format == "Mono16") {
        cv::Mat data_wrapper(cv::Size(img_w, img_h), CV_16U, (void*)data);
        cv::cvtColor(data_wrapper, img_rgb, cv::COLOR_GRAY2RGB);
    }
    
    return img_rgb;
}

// 左相机 stream callback
static void left_stream_callback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer) {
    CameraStreamData* cam_data = (CameraStreamData*)user_data;
    
    switch (type) {
        case ARV_STREAM_CALLBACK_TYPE_INIT:
            if (!!arv_make_thread_high_priority(-10))
                ROS_WARN("Failed to make left stream thread high priority!");
            break;
            
        case ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE: {
            g_assert(buffer == arv_stream_pop_buffer(cam_data->stream));
            g_assert(buffer != NULL);
            
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                uint64_t sys_timestamp = arv_buffer_get_system_timestamp(buffer);
                cv::Mat img_rgb = process_buffer_to_rgb(buffer, cam_data->pixel_format);
                
                if (!img_rgb.empty()) {
                    g_stereo_sync.on_left_frame(img_rgb, sys_timestamp);
                }
            }
            
            arv_stream_push_buffer(cam_data->stream, buffer);
            cam_data->counter++;
            break;
        }
        
        case ARV_STREAM_CALLBACK_TYPE_START_BUFFER:
        case ARV_STREAM_CALLBACK_TYPE_EXIT:
            break;
    }
}

// 右相机 stream callback
static void right_stream_callback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer) {
    CameraStreamData* cam_data = (CameraStreamData*)user_data;
    
    switch (type) {
        case ARV_STREAM_CALLBACK_TYPE_INIT:
            if (!!arv_make_thread_high_priority(-10))
                ROS_WARN("Failed to make right stream thread high priority!");
            break;
            
        case ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE: {
            g_assert(buffer == arv_stream_pop_buffer(cam_data->stream));
            g_assert(buffer != NULL);
            
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                uint64_t sys_timestamp = arv_buffer_get_system_timestamp(buffer);
                cv::Mat img_rgb = process_buffer_to_rgb(buffer, cam_data->pixel_format);
                
                if (!img_rgb.empty()) {
                    g_stereo_sync.on_right_frame(img_rgb, sys_timestamp);
                }
            }
            
            arv_stream_push_buffer(cam_data->stream, buffer);
            cam_data->counter++;
            break;
        }
        
        case ARV_STREAM_CALLBACK_TYPE_START_BUFFER:
        case ARV_STREAM_CALLBACK_TYPE_EXIT:
            break;
    }
}

// 设置枚举参数
static void set_enum_params(ArvCamera* camera, const std::string& feature_name, 
                           const std::string& set_value, GError** error) {
    guint n_enum_avaliables;
    GError* local_error = NULL;
    const char** avaliables = arv_camera_dup_available_enumerations_as_strings(
        camera, feature_name.c_str(), &n_enum_avaliables, &local_error);
    
    if (local_error != NULL) {
        ROS_ERROR("enum %s not supported!", feature_name.c_str());
        g_clear_error(&local_error);
        return;
    }
    
    bool found_desired = false;
    std::string enum_avaliables_str = "";
    for (guint i = 0; i < n_enum_avaliables; ++i) {
        std::string cur = avaliables[i];
        enum_avaliables_str += cur;
        if (i < n_enum_avaliables - 1) enum_avaliables_str += ", ";
        if (cur == set_value) found_desired = true;
    }
    
    ROS_INFO("Camera supported %s: %s", feature_name.c_str(), enum_avaliables_str.c_str());
    if (found_desired) {
        arv_camera_set_string(camera, feature_name.c_str(), set_value.c_str(), error);
        ROS_INFO("Set %s: %s", feature_name.c_str(), set_value.c_str());
    } else {
        ROS_ERROR("Camera does not support %s!", set_value.c_str());
    }
    
    g_free(avaliables);
}

static void set_camera_int(ArvCamera* camera, const std::string& feature, int value, GError** error) {
    int pre_value = arv_camera_get_integer(camera, feature.c_str(), error);
    arv_camera_set_integer(camera, feature.c_str(), value, error);
    ROS_INFO("Set %s: from %d to %d", feature.c_str(), pre_value, value);
}

static void set_camera_float(ArvCamera* camera, const std::string& feature, float value, GError** error) {
    float pre_value = arv_camera_get_float(camera, feature.c_str(), error);
    arv_camera_set_float(camera, feature.c_str(), value, error);
    ROS_INFO("Set %s: from %lf to %lf", feature.c_str(), pre_value, value);
}

// 配置单个相机参数
static void configure_camera(ArvCamera* camera, const std::string& side, GError** error) {
    ROS_INFO("Configuring %s camera...", side.c_str());
    
    // pixel_format
    if (params.pixel_format != "None") 
        set_enum_params(camera, "PixelFormat", params.pixel_format, error);
    
    // Binning
    if (params.binning_h != -1) 
        set_camera_int(camera, "BinningVertical", params.binning_h, error);
    if (params.binning_w != -1) 
        set_camera_int(camera, "BinningHorizontal", params.binning_w, error);
    
    // Exposure
    if (params.exposure_auto != "None") 
        set_enum_params(camera, "ExposureAuto", params.exposure_auto, error);
    if (params.exposure_time > 0) 
        set_camera_float(camera, "ExposureTime", (float)params.exposure_time, error);
    
    // Gain
    if (params.gain_auto != "None") 
        set_enum_params(camera, "GainAuto", params.gain_auto, error);
    if (params.gain > 0) 
        set_camera_float(camera, "Gain", (float)params.gain, error);
    
    // Palette (Twin Thermal Only)
    if (params.palette != "None") 
        set_enum_params(camera, "Palette", params.palette, error);
    
    // GEV 参数
    if (params.gev_packet_size != -1) 
        set_camera_int(camera, "GevSCPSPacketSize", params.gev_packet_size, error);
    if (params.gev_packet_delay != -1) 
        set_camera_int(camera, "GevSCPD", params.gev_packet_delay, error);
    
    // AutoCalTime (自动校准时间)
    if (params.auto_cal_time >= 0) 
        set_camera_int(camera, "AutoCalTime", params.auto_cal_time, error);
    
    // External sync
    bool can_esync = arv_camera_is_feature_available(camera, "ExternalSyncEnable", error);
    if (can_esync && params.externalsync) {
        ROS_WARN("Setting External Sync for %s camera...", side.c_str());
        set_enum_params(camera, "ExternalSyncEnable", "SLAVE", error);
        ros::Duration(0.5).sleep();
    }
    
    // Trigger mode
    bool can_trigger = arv_camera_is_feature_available(camera, "TriggerMode", error);
    if (can_trigger) {
        ROS_WARN("Setting Trigger Mode for %s camera...", side.c_str());
        std::string trigger_str = params.triggermode ? "On" : "Off";
        set_enum_params(camera, "TriggerMode", trigger_str, error);
        ros::Duration(0.5).sleep();
    }
}

// 打开单个相机
static bool open_camera(const std::string& ip, CameraStreamData& cam_data, const std::string& side) {
    ROS_INFO("Opening %s camera at %s...", side.c_str(), ip.c_str());
    
    for (int retry = 0; retry < params.retry_count; ++retry) {
        if (retry > 0) {
            ROS_WARN("Retry %d/%d for %s camera, waiting %d ms...", 
                     retry, params.retry_count, side.c_str(), params.retry_delay_ms);
            ros::Duration(params.retry_delay_ms / 1000.0).sleep();
        }
        
        GError* open_error = NULL;
        cam_data.camera = arv_camera_new(ip.c_str(), &open_error);
        
        if (open_error != NULL) {
            ROS_WARN("Failed to open %s camera: %s", side.c_str(), open_error->message);
            g_clear_error(&open_error);
            cam_data.camera = NULL;
            continue;
        }
        
        if (ARV_IS_CAMERA(cam_data.camera)) {
            cam_data.side = side;
            ROS_INFO("Successfully connected to %s camera at %s", side.c_str(), ip.c_str());
            return true;
        } else {
            ROS_WARN("arv_camera_new returned non-camera object for %s", side.c_str());
            if (cam_data.camera != NULL) {
                g_clear_object(&cam_data.camera);
                cam_data.camera = NULL;
            }
        }
    }
    
    ROS_ERROR("Failed to open %s camera after %d retries!", side.c_str(), params.retry_count);
    return false;
}

// 为相机创建 stream
static bool create_stream(CameraStreamData& cam_data, ArvStreamCallback callback) {
    ROS_INFO("Creating stream for %s camera...", cam_data.side.c_str());
    
    for (int retry = 0; retry < params.retry_count; ++retry) {
        if (retry > 0) {
            ROS_WARN("Stream creation retry %d/%d for %s camera...", 
                     retry, params.retry_count, cam_data.side.c_str());
            ros::Duration(params.retry_delay_ms / 1000.0).sleep();
        }
        
        GError* stream_error = NULL;
        cam_data.stream = arv_camera_create_stream(cam_data.camera, callback, &cam_data, &stream_error);
        
        if (stream_error != NULL) {
            ROS_WARN("Failed to create stream for %s: %s", cam_data.side.c_str(), stream_error->message);
            g_clear_error(&stream_error);
            continue;
        }
        
        if (ARV_IS_STREAM(cam_data.stream)) {
            g_object_set(cam_data.stream,
                "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                "socket-buffer-size", 0,
                NULL);
            
            // 获取 payload 并创建 buffer
            GError* payload_error = NULL;
            cam_data.payload = arv_camera_get_payload(cam_data.camera, &payload_error);
            if (payload_error != NULL) {
                ROS_ERROR("Failed to get payload for %s: %s", cam_data.side.c_str(), payload_error->message);
                g_clear_error(&payload_error);
                g_clear_object(&cam_data.stream);
                cam_data.stream = NULL;
                continue;
            }
            
            // 获取像素格式
            cam_data.pixel_format = arv_camera_get_pixel_format_as_string(cam_data.camera, &payload_error);
            if (payload_error != NULL) {
                ROS_WARN("Failed to get pixel format for %s", cam_data.side.c_str());
                g_clear_error(&payload_error);
            }
            
            // 添加 buffer
            for (int i = 0; i < params.buffer_num; i++) {
                arv_stream_push_buffer(cam_data.stream, arv_buffer_new(cam_data.payload, NULL));
            }
            
            ROS_INFO("Stream created for %s camera, payload=%zu, format=%s", 
                     cam_data.side.c_str(), cam_data.payload, cam_data.pixel_format.c_str());
            return true;
        } else {
            ROS_WARN("Stream object is not valid for %s", cam_data.side.c_str());
            if (cam_data.stream != NULL) {
                g_clear_object(&cam_data.stream);
                cam_data.stream = NULL;
            }
        }
    }
    
    ROS_ERROR("Failed to create stream for %s camera after %d retries!", 
              cam_data.side.c_str(), params.retry_count);
    return false;
}

// 清理单个相机资源
static void cleanup_camera(CameraStreamData& cam_data) {
    if (cam_data.camera != NULL && ARV_IS_CAMERA(cam_data.camera)) {
        GError* stop_error = NULL;
        arv_camera_stop_acquisition(cam_data.camera, &stop_error);
        if (stop_error != NULL) {
            ROS_WARN("Error stopping %s camera: %s", cam_data.side.c_str(), stop_error->message);
            g_clear_error(&stop_error);
        }
    }
    
    if (cam_data.stream != NULL) {
        g_clear_object(&cam_data.stream);
        cam_data.stream = NULL;
    }
    
    if (cam_data.camera != NULL) {
        g_clear_object(&cam_data.camera);
        cam_data.camera = NULL;
    }
}

// 统一资源清理
void cleanup_resources() {
    std::lock_guard<std::mutex> lock(g_cleanup_mutex);
    
    if (g_cleanup_done.exchange(true)) {
        ROS_INFO("Cleanup already performed, skipping...");
        return;
    }
    
    ROS_INFO("Starting stereo resource cleanup...");
    
    // 打印同步统计
    g_stereo_sync.print_stats();
    
    // 清理左相机
    ROS_INFO("Cleaning up left camera...");
    cleanup_camera(g_left_camera);
    
    // 清理右相机
    ROS_INFO("Cleaning up right camera...");
    cleanup_camera(g_right_camera);
    
    // 等待线程退出
    ros::Duration(CLEANUP_DELAY_MS / 1000.0).sleep();
    
    // Aravis shutdown
    ROS_INFO("Shutting down Aravis...");
    arv_shutdown();
    
    // OpenCV 窗口
    cv::destroyAllWindows();
    
    ROS_INFO("Stereo resource cleanup completed.");
}

// 信号处理
void signal_handler(int sig) {
    const char* sig_name = "UNKNOWN";
    switch(sig) {
        case SIGINT:  sig_name = "SIGINT";  break;
        case SIGHUP:  sig_name = "SIGHUP";  break;
        case SIGTERM: sig_name = "SIGTERM"; break;
    }
    ROS_WARN("Received signal %s (%d), initiating cleanup...", sig_name, sig);
    
    cleanup_resources();
    ros::shutdown();
    
    if (sig == SIGTERM || sig == SIGHUP) {
        _exit(0);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_camera", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGHUP, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // ========== 参数读取 ==========
    // 双目相机 IP
    private_nh.param<std::string>("camera_ip_left", params.camera_ip_left, "");
    private_nh.param<std::string>("camera_ip_right", params.camera_ip_right, "");
    
    // 发布话题
    private_nh.param<std::string>("publish_topic_left", params.publish_topic_left, "thermal_left");
    private_nh.param<std::string>("publish_topic_right", params.publish_topic_right, "thermal_right");
    
    // 同步配置
    private_nh.param<std::string>("sync_strategy", params.sync_strategy, "first");
    private_nh.param<int>("sync_timeout_ms", params.sync_timeout_ms, 25);  // 默认25ms
    
    // 共用相机参数
    private_nh.param<std::string>("pixel_format", params.pixel_format, "None");
    private_nh.param<std::string>("palette", params.palette, "None");
    private_nh.param<int>("binning_h", params.binning_h, -1);
    private_nh.param<int>("binning_w", params.binning_w, -1);
    private_nh.param<std::string>("exposure_auto", params.exposure_auto, "None");
    private_nh.param<double>("exposure_time", params.exposure_time, -1.0);
    private_nh.param<std::string>("gain_auto", params.gain_auto, "None");
    private_nh.param<double>("gain", params.gain, -1.0);
    private_nh.param<bool>("vis_img", params.vis_img, false);
    private_nh.param<bool>("externalsync", params.externalsync, false);
    private_nh.param<bool>("triggermode", params.triggermode, false);
    private_nh.param<int>("gev_packet_size", params.gev_packet_size, -1);
    private_nh.param<int>("gev_packet_delay", params.gev_packet_delay, -1);
    private_nh.param<int>("buffer_num", params.buffer_num, 2);
    private_nh.param<int>("retry_count", params.retry_count, DEFAULT_RETRY_COUNT);
    private_nh.param<int>("retry_delay_ms", params.retry_delay_ms, DEFAULT_RETRY_DELAY_MS);
    private_nh.param<int>("auto_cal_time", params.auto_cal_time, 15);
    
    // PTP 触发同步参数
    bool use_ptp_trigger = false;
    private_nh.param<bool>("use_ptp_trigger", use_ptp_trigger, false);
    g_use_ptp_trigger.store(use_ptp_trigger);
    
    // 参数检查
    if (params.camera_ip_left.empty() || params.camera_ip_right.empty()) {
        ROS_ERROR("Both camera_ip_left and camera_ip_right must be specified!");
        return -1;
    }
    
    ROS_INFO("========== Stereo Camera Node ==========");
    ROS_INFO("Left camera IP:  %s", params.camera_ip_left.c_str());
    ROS_INFO("Right camera IP: %s", params.camera_ip_right.c_str());
    ROS_INFO("Left topic:  %s", params.publish_topic_left.c_str());
    ROS_INFO("Right topic: %s", params.publish_topic_right.c_str());
    ROS_INFO("Sync strategy: %s", params.sync_strategy.c_str());
    ROS_INFO("Sync timeout: %d ms", params.sync_timeout_ms);
    ROS_INFO("Use PTP trigger: %s", use_ptp_trigger ? "YES" : "NO");
    ROS_INFO("=========================================");
    
    // ========== 创建 Publisher ==========
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_left = it.advertise(params.publish_topic_left, 5);
    image_transport::Publisher pub_right = it.advertise(params.publish_topic_right, 5);
    
    // ========== 订阅 PTP 触发时间戳 ==========
    ros::Subscriber sub_trigger_time;
    if (use_ptp_trigger) {
        sub_trigger_time = nh.subscribe("/sync/trigger_time", 10, trigger_time_callback);
        ROS_INFO("Subscribed to /sync/trigger_time for PTP trigger synchronization");
    }
    
    // 配置帧同步器
    g_stereo_sync.set_publishers(&pub_left, &pub_right);
    g_stereo_sync.set_sync_strategy(params.sync_strategy);
    g_stereo_sync.set_max_pair_delay(params.sync_timeout_ms * CONVERT_MICRO);  // ms to ns
    
    // ========== 初始化延时 ==========
    ros::Duration(DISCOVERY_DELAY_MS / 1000.0).sleep();
    
    // ========== 打开左相机 ==========
    if (!open_camera(params.camera_ip_left, g_left_camera, "left")) {
        cleanup_resources();
        return -1;
    }
    
    // ========== 打开右相机 ==========
    if (!open_camera(params.camera_ip_right, g_right_camera, "right")) {
        cleanup_resources();
        return -1;
    }
    
    // ========== 配置相机 ==========
    GError* config_error = NULL;
    
    configure_camera(g_left_camera.camera, "left", &config_error);
    if (config_error != NULL) {
        ROS_ERROR("Failed to configure left camera: %s", config_error->message);
        g_clear_error(&config_error);
    }
    
    configure_camera(g_right_camera.camera, "right", &config_error);
    if (config_error != NULL) {
        ROS_ERROR("Failed to configure right camera: %s", config_error->message);
        g_clear_error(&config_error);
    }
    
    // ========== 打印相机信息 ==========
    ROS_INFO("Left camera model: %s", arv_camera_get_model_name(g_left_camera.camera, NULL));
    ROS_INFO("Right camera model: %s", arv_camera_get_model_name(g_right_camera.camera, NULL));
    
    // ========== 设置采集模式 ==========
    arv_camera_set_acquisition_mode(g_left_camera.camera, ARV_ACQUISITION_MODE_CONTINUOUS, &config_error);
    arv_camera_set_acquisition_mode(g_right_camera.camera, ARV_ACQUISITION_MODE_CONTINUOUS, &config_error);
    
    // ========== 创建 Stream ==========
    if (!create_stream(g_left_camera, left_stream_callback)) {
        cleanup_resources();
        return -1;
    }
    
    if (!create_stream(g_right_camera, right_stream_callback)) {
        cleanup_resources();
        return -1;
    }
    
    // ========== 启动采集 ==========
    GError* start_error = NULL;
    
    // 先启动两个相机的 acquisition，尽量同时
    arv_camera_start_acquisition(g_left_camera.camera, &start_error);
    if (start_error != NULL) {
        ROS_ERROR("Failed to start left camera acquisition: %s", start_error->message);
        cleanup_resources();
        return -1;
    }
    
    arv_camera_start_acquisition(g_right_camera.camera, &start_error);
    if (start_error != NULL) {
        ROS_ERROR("Failed to start right camera acquisition: %s", start_error->message);
        cleanup_resources();
        return -1;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("Stereo acquisition started successfully!");
    ROS_INFO("Press Ctrl+C to stop.");
    ROS_INFO("========================================");
    
    // ========== ROS 事件循环 ==========
    ros::spin();
    
    // 清理
    cleanup_resources();
    
    return 0;
}

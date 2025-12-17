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

const uint64_t CONVERT_NANO = 1000000000L;

// 重试配置常量
const int MAX_RETRY_COUNT = 5;
const int RETRY_DELAY_MS = 500;
const int CLEANUP_DELAY_MS = 300;
const int DISCOVERY_DELAY_MS = 200;

std::set<std::string>supported_format = {"YUV422_8", "BayerRG8", "Mono8", "Mono16"};

// 全局清理状态标志
static std::atomic<bool> g_cleanup_done(false);
static std::mutex g_cleanup_mutex;

struct cam_params {
    std::string identify_str;  // 保留兼容，但不再用于设备发现
    std::string camera_ip;     // 新增：相机 IP 直连地址
    std::string pixel_format;
    std::string palette;
    std::string publish_topic;
    int binning_h, binning_w;
    std::string exposure_auto;
    double exposure_time;
    std::string gain_auto;
    double gain;
    bool vis_img;
    bool use_device_time;
    bool enable_ptp;
    bool externalsync;
    bool triggermode;
    int gev_packet_size, gev_packet_delay;
    int buffer_num;
    int tai_ahead;
    int retry_count;           // 新增：重试次数配置
    int retry_delay_ms;        // 新增：重试延时配置
} params;


typedef struct {
	ArvStream *stream;
	size_t counter;
    size_t payload;
    std::string pixel_format;
	gboolean done;
    cam_params* params;
    image_transport::Publisher* pub;
} ArvStreamCallbackData;

void process_image_buffer(ArvBuffer *buffer, ArvStreamCallbackData* callback_data){
    // printf ("Acquired %d×%d buffer\n",
    if(supported_format.find(callback_data->pixel_format) != supported_format.end()){
        int img_w = arv_buffer_get_image_width(buffer);
        int img_h = arv_buffer_get_image_height(buffer);
        size_t data_size;
        const void* data = arv_buffer_get_image_data(buffer, &data_size);
        uint64_t device_time_stamp = arv_buffer_get_timestamp(buffer) - (uint64_t)callback_data->params->tai_ahead * CONVERT_NANO;
        uint64_t system_time_stamp = arv_buffer_get_system_timestamp(buffer);
        uint64_t time_stamp = callback_data->params->use_device_time ? device_time_stamp: system_time_stamp;
        // ROS_INFO("time_diff: %ld",device_time_stamp - system_time_stamp);
        // uint64_t time_stamp = system_time_stamp;
        // ROS_INFO("device_time_stamp: %ld", device_time_stamp);
        cv::Mat img_rgb;
        
        if(callback_data->pixel_format == "YUV422_8"){
            cv::Mat data_warpper(cv::Size(img_w, img_h),CV_8UC2,(void*)data);
            cv::cvtColor(data_warpper,img_rgb,cv::COLOR_YUV2RGB_YUY2);    
        } else if(callback_data->pixel_format == "BayerRG8"){
            cv::Mat data_warpper(cv::Size(img_w, img_h), CV_8U, (void*)data);
            cv::cvtColor(data_warpper, img_rgb,cv::COLOR_BayerRG2BGR);
        } else if(callback_data->pixel_format == "Mono8"){
            cv::Mat data_warpper(cv::Size(img_w, img_h), CV_8U, (void*)data);
            cv::cvtColor(data_warpper, img_rgb, cv::COLOR_GRAY2RGB);
        } else if(callback_data->pixel_format == "Mono16"){
            cv::Mat data_warpper(cv::Size(img_w, img_h), CV_16U, (void*)data);
            cv::cvtColor(data_warpper, img_rgb, cv::COLOR_GRAY2RGB);
        }
        if(callback_data->params->vis_img){
            cv::imshow("show_img",img_rgb);
            cv::waitKey(1);
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"rgb8",img_rgb).toImageMsg();
        msg->header.stamp = ros::Time().fromNSec(time_stamp);

        callback_data->pub->publish(msg);

    } else {
        ROS_ERROR("not supported pixel format:%s", callback_data->pixel_format.c_str());
    }
}

static void stream_callback (void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer) {
	ArvStreamCallbackData *callback_data = (ArvStreamCallbackData *) user_data;

	/* This code is called from the stream receiving thread, which means all the time spent there is less time
	 * available for the reception of incoming packets */

	switch (type) {
		case ARV_STREAM_CALLBACK_TYPE_INIT:
			/* Stream thread started.
			 *
			 * Here you may want to change the thread priority arv_make_thread_realtime() or
			 * arv_make_thread_high_priority() */
            if (!!arv_make_thread_high_priority(-10))
                ROS_WARN("Failed to make stream thread high priority!");
			break;
		case ARV_STREAM_CALLBACK_TYPE_START_BUFFER:
			/* The first packet of a new frame was received */
			break;
		case ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE: {
			/* The buffer is received, successfully or not. It is already pushed in the output FIFO.
			 *
			 * You could here signal the new buffer to another thread than the main one, and pull/push the
			 * buffer from this another thread.
			 *
			 * Or use the buffer here. We need to pull it, process it, then push it back for reuse by the
			 * stream receiving thread */

			g_assert (buffer == arv_stream_pop_buffer(callback_data->stream));
			g_assert (buffer != NULL);

			// /* Retrieve 10 buffers */
			// if (callback_data->counter < 10) {
            uint64_t device_time_stamp = 0;
            uint64_t system_time_stamp = 0;
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS){
                system_time_stamp = arv_buffer_get_system_timestamp(buffer);
                device_time_stamp = arv_buffer_get_timestamp(buffer) - (uint64_t)callback_data->params->tai_ahead * CONVERT_NANO;
                process_image_buffer(buffer, callback_data);
            }

            arv_stream_push_buffer(callback_data->stream, buffer);
            callback_data->counter++;
            
            // calc fps & bandwidith
            static auto t_last_calc_fps = std::chrono::steady_clock::now();
            auto t_now = std::chrono::steady_clock::now();
            double duration = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_last_calc_fps).count() / 1e6;
            if (duration > 5.0){
               double fps = callback_data->counter / duration;
               double bandwidth = callback_data->payload * fps / 1024.0 / 1024.0;
               ROS_INFO("fps: %lf, bandwidth: %lf MB/s", fps, bandwidth);
               callback_data->counter = 0;
               t_last_calc_fps = t_now;

               if(callback_data->params->use_device_time){
                    // ROS_INFO("%ld %ld",device_time_stamp, system_time_stamp);
                    ROS_INFO("system time behind: %.4lf ms",(system_time_stamp/(double)1e6) - (device_time_stamp/(double)1e6));
               }
            }
			
			break;
        }
		case ARV_STREAM_CALLBACK_TYPE_EXIT:
			/* Stream thread ended */
			break;
	}
}

// arv_camera_dup_available_components

static void set_enum_params(ArvCamera* camera, std::string feature_name, std::string set_value, GError** error){
    guint n_enum_avaliables;
    GError** local_error = NULL;
    const char** avaliables = arv_camera_dup_available_enumerations_as_strings(camera, feature_name.c_str(), &n_enum_avaliables, local_error);
    if(local_error != NULL){
        ROS_ERROR("enum %s not supported!",feature_name.c_str());
        return;
    }
    bool found_desired = false;
    std::string enum_avaliables_str = "";
    for(int i = 0;i < n_enum_avaliables; ++i){
        std::string cur = avaliables[i];
        enum_avaliables_str += cur;
        if(i < n_enum_avaliables - 1) enum_avaliables_str += ", ";
        if(cur == set_value) found_desired = true;
    }
    ROS_INFO("camera supported %s: %s",feature_name.c_str(),enum_avaliables_str.c_str());
    if(found_desired){
        arv_camera_set_string(camera, feature_name.c_str(), set_value.c_str(), error);
        ROS_INFO("set %s: %s",feature_name.c_str(),set_value.c_str());
    } else {
        ROS_ERROR("camera does not support %s!",set_value.c_str());
    }
}

static void set_camera_int(ArvCamera* camera, std::string feature, int value, GError** error){
    int pre_value = arv_camera_get_integer(camera, feature.c_str(), error);
    arv_camera_set_integer(camera, feature.c_str(), value, error);
    ROS_INFO("set %s: from %d to %d",feature.c_str(), pre_value, value);
}

static void set_camera_float(ArvCamera* camera, std::string feature, float value, GError** error){
    float pre_value = arv_camera_get_float(camera, feature.c_str(), error);
    arv_camera_set_float(camera, feature.c_str(), value, error);
    ROS_INFO("set %s: from %lf to %lf",feature.c_str(), pre_value, value);
}

static void update_camera_params(ArvCamera* camera, GError** error){
    // pixel_format
    if(params.pixel_format != "None") set_enum_params(camera,"PixelFormat",params.pixel_format,error);
    // Binning
    if(params.binning_h != -1) set_camera_int(camera, "BinningVertical", params.binning_h, error);
    if(params.binning_w != -1) set_camera_int(camera, "BinningHorizontal", params.binning_w, error);
    // Exposure
    if(params.exposure_auto != "None") set_enum_params(camera, "ExposureAuto", params.exposure_auto, error);
    if(params.exposure_time > 0) set_camera_float(camera, "ExposureTime", (float)params.exposure_time, error);
    // Gain
    if(params.gain_auto != "None") set_enum_params(camera, "GainAuto", params.gain_auto, error);
    if(params.gain > 0) set_camera_float(camera, "Gain", (float)params.gain, error);
    // Palette (Twin Thermal Only)
    if(params.palette != "None") set_enum_params(camera, "Palette", params.palette, error);
    if(params.gev_packet_size != -1) set_camera_int(camera, "GevSCPSPacketSize", params.gev_packet_size, error);
    if(params.gev_packet_delay != -1) set_camera_int(camera, "GevSCPD", params.gev_packet_delay, error);
    // PTP
    bool can_ptp = arv_camera_is_feature_available(camera, "GevIEEE1588", error);
    if(can_ptp){
        arv_camera_set_boolean(camera,"GevIEEE1588",false,error);
        ros::Duration(0.2).sleep();
    }
    if(params.enable_ptp){
        arv_camera_set_boolean(camera,"GevIEEE1588",true,error);
        // arv_camera_set_boolean(camera,"GevIEEE1588SlaveOnly",true,error);
        ROS_WARN("Eanbling camera PTP...");
        ros::Duration(7.0).sleep();
    }
    //external sync
    bool can_esync = arv_camera_is_feature_available(camera, "ExternalSyncEnable", error);
    if(can_esync){
        ROS_WARN("Setting External Sync...");
        std::string esync_str = params.externalsync ? "SLAVE": "OFF";
        set_enum_params(camera,"ExternalSyncEnable",esync_str,error);
        ros::Duration(0.5).sleep();
    }
    //external trigger
    bool can_trigger = arv_camera_is_feature_available(camera, "TriggerMode", error);
    if(can_trigger){
        ROS_WARN("Setting Trigger Sync...");
        std::string trigger_str = params.triggermode ? "On": "Off";
        set_enum_params(camera,"TriggerMode",trigger_str,error);
        ros::Duration(0.5).sleep();
    }
}

GError* error = NULL;
ArvCamera* camera = NULL;
ArvDevice* camera_device = NULL;
ArvStreamCallbackData callback_data;

/**
 * 统一的资源清理函数
 * 确保在任何退出路径中都能正确释放资源
 * 清理顺序严格遵循：stop acquisition -> wait -> stream -> camera -> device -> aravis shutdown -> opencv
 */
void cleanup_resources() {
    std::lock_guard<std::mutex> lock(g_cleanup_mutex);
    
    // 防止重复清理
    if (g_cleanup_done.exchange(true)) {
        ROS_INFO("Cleanup already performed, skipping...");
        return;
    }
    
    ROS_INFO("Starting resource cleanup...");
    GError* cleanup_error = NULL;
    
    // 1. 停止相机采集
    if (camera != NULL && ARV_IS_CAMERA(camera)) {
        ROS_INFO("Stopping camera acquisition...");
        arv_camera_stop_acquisition(camera, &cleanup_error);
        if (cleanup_error != NULL) {
            ROS_WARN("Error stopping acquisition: %s", cleanup_error->message);
            g_clear_error(&cleanup_error);
        }
    }
    
    // 2. 等待 UDP/stream 线程退出
    ROS_INFO("Waiting for stream threads to exit...");
    ros::Duration(CLEANUP_DELAY_MS / 1000.0).sleep();
    
    // 3. 释放 stream 对象
    if (callback_data.stream != NULL) {
        ROS_INFO("Releasing stream object...");
        g_clear_object(&callback_data.stream);
        callback_data.stream = NULL;
    }
    
    // 4. 释放 camera 对象
    if (camera != NULL) {
        ROS_INFO("Releasing camera object...");
        g_clear_object(&camera);
        camera = NULL;
    }
    
    // 5. device 对象由 camera 管理，不需要单独释放
    // 注意：arv_camera_get_device 返回的是内部引用，camera 释放时会自动处理
    camera_device = NULL;
    
    // 6. 调用 Aravis shutdown
    ROS_INFO("Shutting down Aravis...");
    arv_shutdown();
    
    // 7. 关闭 OpenCV 窗口
    ROS_INFO("Closing OpenCV windows...");
    cv::destroyAllWindows();
    
    ROS_INFO("Resource cleanup completed.");
}

/**
 * 统一的信号处理函数
 * 处理 SIGINT (Ctrl+C), SIGHUP (终端关闭), SIGTERM (roslaunch stop)
 */
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
    
    // 对于 SIGTERM/SIGHUP，确保进程退出
    if (sig == SIGTERM || sig == SIGHUP) {
        _exit(0);
    }
}

/**
 * ROS shutdown 回调
 * 确保 ROS 正常 shutdown 时也执行清理
 */
void ros_shutdown_callback(int sig) {
    ROS_INFO("ROS shutdown callback triggered.");
    cleanup_resources();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "thermal_camera", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 注册多种信号处理
    signal(SIGINT, signal_handler);
    signal(SIGHUP, signal_handler);
    signal(SIGTERM, signal_handler);

    // 原有参数
    private_nh.param<std::string>("identify_str", params.identify_str, "None");
    private_nh.param<std::string>("pixel_format", params.pixel_format, "None");
    private_nh.param<int>("binning_h", params.binning_h, -1);
    private_nh.param<int>("binning_w", params.binning_w, -1);
    private_nh.param<std::string>("exposure_auto", params.exposure_auto, "None");
    private_nh.param<double>("exposure_time", params.exposure_time, -1.0);
    private_nh.param<std::string>("gain_auto", params.gain_auto, "None");
    private_nh.param<double>("gain", params.gain, -1.0);
    private_nh.param<std::string>("palette", params.palette, "None");
    private_nh.param<bool>("vis_img", params.vis_img, false);
    private_nh.param<std::string>("publish_topic", params.publish_topic, "test_genicam");
    private_nh.param<int>("gev_packet_size",params.gev_packet_size, -1);
    private_nh.param<int>("gev_packet_delay",params.gev_packet_delay, -1);
    private_nh.param<int>("buffer_num",params.buffer_num, 2);
    private_nh.param<bool>("use_device_time", params.use_device_time, false);
    private_nh.param<bool>("enable_ptp", params.enable_ptp, false);
    private_nh.param<bool>("externalsync", params.externalsync, false);
    private_nh.param<bool>("triggermode", params.triggermode, false);
    private_nh.param<int>("tai_ahead", params.tai_ahead, 0);
    
    // 新增参数：IP 直连模式
    private_nh.param<std::string>("camera_ip", params.camera_ip, "");
    private_nh.param<int>("retry_count", params.retry_count, MAX_RETRY_COUNT);
    private_nh.param<int>("retry_delay_ms", params.retry_delay_ms, RETRY_DELAY_MS);
    
    if(params.use_device_time){
        ROS_WARN("Device Timestamp enabled. Make sure the time is synchronized!");
    }

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(params.publish_topic, 5);
    ROS_INFO("publish_topic: %s",params.publish_topic.c_str());
    
    // ========== Aravis 初始化和稳定化 ==========
    ROS_INFO("Initializing Aravis library...");
    
    // 启动前延时，确保之前的实例完全释放
    ros::Duration(DISCOVERY_DELAY_MS / 1000.0).sleep();
    
    bool device_opened = false;
    
    // ========== IP 直连模式（推荐） ==========
    if (!params.camera_ip.empty()) {
        ROS_INFO("Using IP direct connection mode: %s", params.camera_ip.c_str());
        
        // 带重试的设备打开
        for (int retry = 0; retry < params.retry_count && !device_opened; ++retry) {
            if (retry > 0) {
                ROS_WARN("Retry %d/%d: waiting %d ms before reconnect...", 
                         retry, params.retry_count, params.retry_delay_ms);
                ros::Duration(params.retry_delay_ms / 1000.0).sleep();
            }
            
            GError* open_error = NULL;
            
            // 直接通过 IP 打开相机（绕过 device list）
            camera = arv_camera_new(params.camera_ip.c_str(), &open_error);
            
            if (open_error != NULL) {
                ROS_WARN("Failed to open camera at %s: %s", 
                         params.camera_ip.c_str(), open_error->message);
                g_clear_error(&open_error);
                camera = NULL;
                continue;
            }
            
            if (ARV_IS_CAMERA(camera)) {
                camera_device = arv_camera_get_device(camera);
                device_opened = true;
                ROS_INFO("Successfully connected to camera at %s", params.camera_ip.c_str());
            } else {
                ROS_WARN("arv_camera_new returned non-camera object for %s", params.camera_ip.c_str());
                if (camera != NULL) {
                    g_clear_object(&camera);
                    camera = NULL;
                }
            }
        }
        
        if (!device_opened) {
            ROS_ERROR("Failed to open camera at %s after %d retries!", 
                      params.camera_ip.c_str(), params.retry_count);
            cleanup_resources();
            return -1;
        }
    }
    // ========== 兼容模式：使用 identify_str 扫描（不推荐） ==========
    else if (params.identify_str != "None") {
        ROS_WARN("Using legacy device scan mode (identify_str). Consider using camera_ip for better stability.");
        
        for (int retry = 0; retry < params.retry_count && !device_opened; ++retry) {
            if (retry > 0) {
                ROS_WARN("Retry %d/%d: waiting %d ms before rescan...", 
                         retry, params.retry_count, params.retry_delay_ms);
                ros::Duration(params.retry_delay_ms / 1000.0).sleep();
            }
            
            // 刷新设备列表前等待
            ros::Duration(DISCOVERY_DELAY_MS / 1000.0).sleep();
            arv_update_device_list();
            
            int n_devices = arv_get_n_devices();
            ROS_INFO("Device scan attempt %d: found %d devices", retry + 1, n_devices);
            
            int found_device_id = -1;
            for(int i = 0; i < n_devices; ++i){
                std::string device_string = std::string(arv_get_device_id(i)) + " (" + arv_get_device_address(i)+")";
                ROS_INFO("  Device %d: %s", i, device_string.c_str());
                if(device_string.find(params.identify_str) != std::string::npos){
                    ROS_INFO("  -> Matched identify_str: %s", params.identify_str.c_str());
                    found_device_id = i;
                    break;  // 找到第一个匹配的就停止
                }
            }
            
            if(found_device_id != -1){
                GError* open_error = NULL;
                camera_device = arv_open_device(arv_get_device_id(found_device_id), &open_error);
                
                if (open_error != NULL) {
                    ROS_WARN("Failed to open device: %s", open_error->message);
                    g_clear_error(&open_error);
                    continue;
                }
                
                camera = arv_camera_new_with_device(camera_device, &open_error);
                
                if (open_error != NULL) {
                    ROS_WARN("Failed to create camera: %s", open_error->message);
                    g_clear_error(&open_error);
                    if (camera_device != NULL) {
                        g_clear_object(&camera_device);
                        camera_device = NULL;
                    }
                    continue;
                }
                
                if(ARV_IS_CAMERA(camera)){
                    device_opened = true;
                    ROS_INFO("Successfully opened camera: %s", params.identify_str.c_str());
                }
            }
        }
        
        if (!device_opened) {
            ROS_ERROR("Camera %s not found after %d retries!", 
                      params.identify_str.c_str(), params.retry_count);
            cleanup_resources();
            return -1;
        }
    } else {
        ROS_ERROR("No camera_ip or identify_str specified! Please set one of them.");
        cleanup_resources();
        return -1;
    }
    
    // ========== 相机配置和 Stream 创建 ==========
    // 此时 camera 对象已成功创建
    
    int width = 0;
    int height = 0;
    double gain_value = 0.0;
    std::string current_pixel_format = "unKnown";
    
    ROS_INFO("Opening camera: %s", arv_camera_get_model_name(camera, NULL));
    update_camera_params(camera, &error);
    
    if (!error) width = arv_camera_get_integer(camera, "Width", &error);
    if (!error) height = arv_camera_get_integer(camera, "Height", &error);
    if (!error) current_pixel_format = arv_camera_get_pixel_format_as_string(camera, &error);
    if (!error && params.gain > 0) gain_value = arv_camera_get_float(camera, "Gain", &error);
    
    ROS_INFO("Resolution: %d x %d", width, height);
    if (params.gain > 0) ROS_INFO("Gain: %lf", gain_value);
    ROS_INFO("Current pixel format: %s", current_pixel_format.c_str());
    
    // 初始化 callback_data
    callback_data.counter = 0;
    callback_data.pub = &pub;
    callback_data.params = &params;
    callback_data.done = FALSE;
    callback_data.stream = NULL;
    callback_data.pixel_format = arv_camera_get_pixel_format_as_string(camera, &error);
    
    if (error != NULL) {
        ROS_ERROR("Failed to get pixel format: %s", error->message);
        cleanup_resources();
        return -1;
    }
    
    arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
    
    if (error != NULL) {
        ROS_ERROR("Failed to set acquisition mode: %s", error->message);
        cleanup_resources();
        return -1;
    }
    
    // 带重试的 Stream 创建
    bool stream_created = false;
    for (int retry = 0; retry < params.retry_count && !stream_created; ++retry) {
        if (retry > 0) {
            ROS_WARN("Stream creation retry %d/%d, waiting %d ms...", 
                     retry, params.retry_count, params.retry_delay_ms);
            ros::Duration(params.retry_delay_ms / 1000.0).sleep();
        }
        
        GError* stream_error = NULL;
        callback_data.stream = arv_camera_create_stream(camera, stream_callback, &callback_data, &stream_error);
        
        if (stream_error != NULL) {
            ROS_WARN("Failed to create stream: %s", stream_error->message);
            g_clear_error(&stream_error);
            continue;
        }
        
        if (ARV_IS_STREAM(callback_data.stream)) {
            g_object_set(callback_data.stream,
                "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                "socket-buffer-size", 0,
                NULL);
            stream_created = true;
            ROS_INFO("Stream created successfully.");
        } else {
            ROS_WARN("Stream object is not valid.");
            if (callback_data.stream != NULL) {
                g_clear_object(&callback_data.stream);
                callback_data.stream = NULL;
            }
        }
    }
    
    if (!stream_created) {
        ROS_ERROR("Failed to create stream after %d retries!", params.retry_count);
        cleanup_resources();
        return -1;
    }
    
    // 获取 payload 并创建 buffer
    size_t payload = arv_camera_get_payload(camera, &error);
    if (error != NULL) {
        ROS_ERROR("Failed to get payload: %s", error->message);
        cleanup_resources();
        return -1;
    }
    
    ROS_INFO("Payload size: %ld", payload);
    callback_data.payload = payload;
    
    // 向 stream buffer pool 添加 buffer
    for (int i = 0; i < params.buffer_num; i++) {
        arv_stream_push_buffer(callback_data.stream, arv_buffer_new(payload, NULL));
    }
    
    // 启动采集
    arv_camera_start_acquisition(camera, &error);
    if (error != NULL) {
        ROS_ERROR("Failed to start acquisition: %s", error->message);
        cleanup_resources();
        return -1;
    }
    
    ROS_INFO("Acquisition started successfully!");
    ROS_INFO("Camera driver is running. Press Ctrl+C to stop.");
    
    // 进入 ROS 事件循环
    ros::spin();
    
    // 正常退出时的清理（通常由 signal handler 或 on_shutdown 完成）
    cleanup_resources();
    
    return 0;
}
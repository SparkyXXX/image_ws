#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <signal.h>

using namespace std;
using namespace cv;

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CAMERA_DEVICE "/dev/video2"

// 定义结构体来保存帧数据
typedef struct {
    guint8 *data;
    gsize size;
    GstClockTime timestamp;
    GstClockTime absolute_timestamp;
} FrameData;

FrameData* create_frame_data(GstBuffer *buffer, GstClockTime timestamp);
static GstFlowReturn new_sample_cb(GstElement *sink, gpointer data);
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data);
static void signal_handler(int signum);
static GMainLoop *loop = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera2_10005_publisher");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("camera2_10005_image", 10);
    if (!publisher) {
        g_printerr("Error: Failed to create publisher\n");
        return -1;
    }

    signal(SIGINT, signal_handler);

    gst_init(&argc, &argv);

    loop = g_main_loop_new(NULL, FALSE);

    GstElement *pipeline = gst_pipeline_new("usb-camera");
    GstElement *source = gst_element_factory_make("v4l2src", "camera-input");
    GstElement *source_capsfilter = gst_element_factory_make("capsfilter", "source_capsfilter");
    GstElement *jpegdec = gst_element_factory_make("jpegdec", "jpeg-dec");
    GstElement *converter = gst_element_factory_make("videoconvert", "video-converter");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "caps-filter");
    GstElement *sink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !source || !source_capsfilter || !jpegdec || !converter || !capsfilter || !sink) {
        g_printerr("One of the elements could not be created. Exiting.\n");
        return -1;
    }

    g_object_set(G_OBJECT(source), "device", CAMERA_DEVICE, NULL);
    GstCaps *source_caps = gst_caps_new_simple("image/jpeg", "width", G_TYPE_INT, IMAGE_WIDTH, "height", G_TYPE_INT, IMAGE_HEIGHT, "framerate", GST_TYPE_FRACTION, 30, 1, NULL);
    g_object_set(G_OBJECT(source_capsfilter), "caps", source_caps, NULL);
    gst_caps_unref(source_caps);

    GstCaps *caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(sink, "emit-signals", TRUE, "max-buffers", 10, "drop", TRUE, NULL);
    g_signal_connect(sink, "new-sample", G_CALLBACK(new_sample_cb), &publisher);

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), source, source_capsfilter, jpegdec, converter, capsfilter, sink, NULL);
    gst_element_link_many(source, source_capsfilter, jpegdec, converter, capsfilter, sink, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("Running\n");

    g_main_loop_run(loop);

    g_print("Returned, stopping playback\n");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));

    rclcpp::shutdown();
    return 0;
}

// 回调函数，处理从appsink接收到的GstSample
static GstFlowReturn new_sample_cb(GstElement *sink, gpointer data)
{
    auto publisher = static_cast<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr *>(data);
    if (!publisher || !(*publisher)) {
        g_printerr("Error: Publisher pointer is null\n");
        return GST_FLOW_ERROR;
    }

    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;

    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (!sample) {
        g_print("No sample\n");
        return GST_FLOW_ERROR;
    }

    buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_print("No buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    rclcpp::Time ros_now = rclcpp::Clock().now();
    auto total_nanoseconds = ros_now.nanoseconds();
    auto seconds = total_nanoseconds / 1000000000;
    auto nanoseconds = total_nanoseconds % 1000000000;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        cv::Mat frame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, map.data);
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = ros_now;
        msg->header.frame_id = "camera2_10005_frame";
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = frame.cols * frame.channels();
        msg->data.assign(frame.data, frame.data + map.size);
        
        // 发布ROS2消息，打印时间戳
        (*publisher)->publish(std::move(msg));
        g_print("[2]ROS 2 Time: %ld.%09ld\n", seconds, nanoseconds);
        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = static_cast<GMainLoop *>(data);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR:
            {
                gchar *debug;
                GError *error;
                gst_message_parse_error(msg, &error, &debug);
                g_free(debug);
                g_printerr("ERROR:%s\n", error->message);
                g_error_free(error);
                g_main_loop_quit(loop);
                break;
            }
        default:
            break;
    }
    return TRUE;
}

static void signal_handler(int signum)
{
    if (loop != nullptr) {
        g_main_loop_quit(loop);
    }
}

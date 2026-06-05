// BLK360 scanner ROS2 node.
//
// Subscribes to a trigger topic; when the configured command ("sequence") arrives
// it runs the full BLK360 colorize-point-cloud workflow (the logic ported from the
// 22-colorize-pc-ldr sample) and saves a colorized point cloud to a CSV file.
//
// Topics:
//   sub  /blk360/scan_trigger  (std_msgs/String)  -> start scan when data == trigger_command
//   pub  /blk360/scan_status    (std_msgs/String)  -> IDLE | SCANNING | CAPTURED | DONE | ERROR
//        CAPTURED = physical scan finished (robot may move); DONE = data fully downloaded
//   pub  /blk360/scan_progress  (std_msgs/String)  -> human readable progress lines
//
// Parameters:
//   device_address        (string, "192.168.10.90:8081")
//   trigger_command       (string, "scan")
//   output_dir            (string, ".")
//   point_cloud_density   (string, "medium")  -> low | medium | high
//   panorama_mode         (string, "ldr")     -> ldr | hdr

#include "BLK360.h"
#include "blk360_scanner/PC.hxx"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

namespace
{
// Thrown internally whenever the BLK360 API reports an error, so the scan can unwind
// to a single cleanup/error-status point instead of calling exit() like the sample.
struct ScanError : std::runtime_error
{
    explicit ScanError(const std::string& msg) : std::runtime_error(msg) {}
};

std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}
}

class Blk360ScannerNode : public rclcpp::Node
{
public:
    Blk360ScannerNode() : Node("blk360_scanner")
    {
        device_address_ = declare_parameter<std::string>("device_address", "192.168.10.90:8081");
        trigger_command_ = declare_parameter<std::string>("trigger_command", "scan");
        output_dir_ = declare_parameter<std::string>("output_dir", "scans");
        density_param_ = declare_parameter<std::string>("point_cloud_density", "medium");
        panorama_param_ = declare_parameter<std::string>("panorama_mode", "ldr");

        status_pub_ = create_publisher<std_msgs::msg::String>("blk360/scan_status", 10);
        progress_pub_ = create_publisher<std_msgs::msg::String>("blk360/scan_progress", 10);
        trigger_sub_ = create_subscription<std_msgs::msg::String>(
            "blk360/scan_trigger", 10,
            std::bind(&Blk360ScannerNode::onTrigger, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "BLK360 scanner ready. Send std_msgs/String \"%s\" to /blk360/scan_trigger to start. Device=%s, output=%s",
                    trigger_command_.c_str(), device_address_.c_str(), output_dir_.c_str());
        publishStatus("IDLE");
    }

    ~Blk360ScannerNode() override
    {
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

private:
    // ---- ROS callbacks -----------------------------------------------------

    void onTrigger(const std_msgs::msg::String& msg)
    {
        if (msg.data != trigger_command_)
        {
            RCLCPP_DEBUG(get_logger(), "Ignoring trigger '%s' (expecting '%s').",
                         msg.data.c_str(), trigger_command_.c_str());
            return;
        }

        // Reject overlapping scans: the device serves one measurement at a time.
        if (busy_.exchange(true))
        {
            RCLCPP_WARN(get_logger(), "Scan already in progress, ignoring trigger.");
            return;
        }

        // Reap the previous (finished) worker before launching a new one.
        if (worker_.joinable())
        {
            worker_.join();
        }
        worker_ = std::thread([this]()
        {
            runScan();
            busy_.store(false);
        });
    }

    void publishStatus(const std::string& s)
    {
        std_msgs::msg::String m;
        m.data = s;
        status_pub_->publish(m);
    }

    void publishProgress(const std::string& s)
    {
        std_msgs::msg::String m;
        m.data = s;
        progress_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
    }

    // ---- BLK360 helpers ----------------------------------------------------

    // Mirrors the sample's checkError(), but throws instead of exit()-ing so the
    // worker thread can clean up and report ERROR without killing the node.
    void check(const char* step)
    {
        const Blk360_Error error = Blk360_Api_GetLastError();
        if (error.code != Blk360_Error_Ok)
        {
            throw ScanError(std::string(step) + ": " + error.message);
        }
    }

    Blk360_PointCloudDensity densityFromParam() const
    {
        const std::string d = toLower(density_param_);
        if (d == "low")  return Blk360_PointCloudDensity_Low;
        if (d == "high") return Blk360_PointCloudDensity_High;
        return Blk360_PointCloudDensity_Medium;
    }

    Blk360_PanoramaMode panoramaModeFromParam() const
    {
        return toLower(panorama_param_) == "hdr" ? Blk360_PanoramaMode_HDR : Blk360_PanoramaMode_LDR;
    }

    std::ofstream createOutputFile(Blk360_scanId_t scanId)
    {
        // ofstream won't create missing parent directories, so make them first.
        if (!output_dir_.empty())
        {
            std::error_code ec;
            std::filesystem::create_directories(output_dir_, ec);
            if (ec)
            {
                RCLCPP_ERROR(get_logger(), "Could not create output directory %s: %s",
                             output_dir_.c_str(), ec.message().c_str());
                return std::ofstream{};  // not open -> caller reports the failure
            }
        }

        const std::filesystem::path path =
            std::filesystem::path(output_dir_) / ("pointcloud_" + std::to_string(scanId) + ".csv");
        std::ofstream file(path, std::ios_base::out);
        if (file.is_open())
        {
            file << "x [m],y [m],z [m],r,g,b\n";
            RCLCPP_INFO(get_logger(), "Writing point cloud to %s",
                        std::filesystem::absolute(path).c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not open output file: %s", path.c_str());
        }
        return file;
    }

    void cleanup()
    {
        Blk360_PointCloudColorizer_Release(colorizer_);
        Blk360_EventQueue_Release(queue_);
        Blk360_Measurement_Release(measurement_);
        Blk360_ProcessingWorkflow_Release(processingWorkflow_);
        Blk360_DataManipulationWorkflow_Release(dataManipulationWorkflow_);
        Blk360_MeasurementWorkflow_Release(measurementWorkflow_);
        Blk360_SystemWorkflow_Release(systemWorkflow_);
        Blk360_Session_Release(session_);
        Blk360_Api_Release();

        // Handle types are structs wrapping a raw Blk360_Handle, so reset the inner member.
        colorizer_.handle = Blk360_Handle_Null();
        queue_.handle = Blk360_Handle_Null();
        measurement_.handle = Blk360_Handle_Null();
        processingWorkflow_.handle = Blk360_Handle_Null();
        dataManipulationWorkflow_.handle = Blk360_Handle_Null();
        measurementWorkflow_.handle = Blk360_Handle_Null();
        systemWorkflow_.handle = Blk360_Handle_Null();
        session_.handle = Blk360_Handle_Null();
    }

    // ---- Scan workflow -----------------------------------------------------

    void runScan()
    {
        RCLCPP_INFO(get_logger(), "=== BLK360 scan started (device %s) ===", device_address_.c_str());
        publishStatus("SCANNING");
        try
        {
            doScan();
            cleanup();
            RCLCPP_INFO(get_logger(), "=== BLK360 scan finished ===");
            publishStatus("DONE");
        }
        catch (const ScanError& e)
        {
            RCLCPP_ERROR(get_logger(), "Scan failed: %s", e.what());
            cleanup();
            publishStatus(std::string("ERROR: ") + e.what());
        }
    }

    void doScan()
    {
        Blk360_Api_New(BLK360_LIBRARY_VERSION);
        check("Api_New");

        session_ = Blk360_Session_New_Default(device_address_.c_str());
        check("Session_New_Default");

        measurementWorkflow_ = Blk360_MeasurementWorkflow_Create(session_);
        check("MeasurementWorkflow_Create");

        systemWorkflow_ = Blk360_SystemWorkflow_Create(session_);
        check("SystemWorkflow_Create");

        processingWorkflow_ = Blk360_ProcessingWorkflow_Create(session_);
        check("ProcessingWorkflow_Create");

        dataManipulationWorkflow_ = Blk360_DataManipulationWorkflow_Create(session_);
        check("DataManipulationWorkflow_Create");

        queue_ = Blk360_EventQueue_New(100);
        check("EventQueue_New");

        // --- Perform measurement ---
        Blk360_MeasurementParameters parameters = Blk360_MeasurementParameters_New();
        parameters.enablePanorama = true;
        parameters.enablePointCloud = true;
        parameters.enableIr = false;
        parameters.panoramaParameters.panoramaMode = panoramaModeFromParam();
        parameters.pointCloudParameters.density = densityFromParam();

        const Blk360_SubscriptionHandle measurementProgress = Blk360_MeasurementWorkflow_OnProgress(measurementWorkflow_, queue_);
        check("MeasurementWorkflow_OnProgress");

        const Blk360_SubscriptionHandle measurementError = Blk360_MeasurementWorkflow_OnError(measurementWorkflow_, queue_);
        check("MeasurementWorkflow_OnError");

        const Blk360_scanId_t scanId = Blk360_MeasurementWorkflow_Start(measurementWorkflow_, parameters);
        check("MeasurementWorkflow_Start");

        publishProgress("Assigned scan id: " + std::to_string(scanId));

        while (Blk360_EventQueue_Wait(queue_, 20000))
        {
            const Blk360_Event baseEvent = Blk360_EventQueue_Pop(queue_);
            check("EventQueue_Pop (measurement)");

            if (baseEvent.sender.handle == measurementProgress.handle)
            {
                const auto& progressEvent = reinterpret_cast<const Blk360_ProgressEvent&>(baseEvent);
                publishProgress("Image panorama progress " + std::to_string(progressEvent.imagesDone) +
                                " out of " + std::to_string(progressEvent.imagesTotal));

                if (progressEvent.imagesDone == progressEvent.imagesTotal)
                {
                    break;  // all images captured, ready to colorize
                }
            }
            else if (baseEvent.sender.handle == measurementError.handle)
            {
                throwEventError(baseEvent, "measurement");
            }
            else
            {
                throw ScanError("Unexpected event during measurement.");
            }
        }

        publishProgress("Image panorama for scan " + std::to_string(scanId) + " finished.");

        // The physical capture is done here: the BLK360 no longer needs the robot
        // to stay still. Everything below (download + colorize + point-cloud
        // processing) is pure data transfer, so the orchestrator can resume
        // driving while it runs. CAPTURED marks that hand-off; DONE still fires at
        // the very end once the data is fully downloaded and written.
        publishStatus("CAPTURED");

        // Stop listening for measurement events, restart a larger queue for downloads.
        Blk360_EventQueue_Release(queue_);
        check("EventQueue_Release (measurement)");
        queue_ = Blk360_EventQueue_New(1000);

        // --- Download panorama ---
        measurement_ = Blk360_DataManipulationWorkflow_GetMeasurementByScanId(dataManipulationWorkflow_, scanId);
        if (Blk360_Handle_IsNull(measurement_.handle))
        {
            throw ScanError("Could not find measurement with given id.");
        }

        const Blk360_SubscriptionHandle imageDownloadProgressEvents = Blk360_DataManipulationWorkflow_OnProgress(dataManipulationWorkflow_, queue_);
        check("DataManipulationWorkflow_OnProgress");

        const Blk360_SubscriptionHandle imageDownloadErrorEvents = Blk360_DataManipulationWorkflow_OnError(dataManipulationWorkflow_, queue_);
        check("DataManipulationWorkflow_OnError");

        Blk360_DataManipulationWorkflow_DownloadPanorama(dataManipulationWorkflow_, measurement_);
        check("DownloadPanorama");

        while (Blk360_EventQueue_Wait(queue_, 20000))
        {
            const Blk360_Event baseEvent = Blk360_EventQueue_Pop(queue_);
            check("EventQueue_Pop (panorama download)");

            if (baseEvent.sender.handle == imageDownloadProgressEvents.handle)
            {
                const auto& event = reinterpret_cast<const Blk360_ProgressEvent&>(baseEvent);
                const auto imagesDone = event.imagesDone;
                if (imagesDone == 0)
                {
                    continue;
                }

                publishProgress("Downloaded images: " + std::to_string(imagesDone));

                Blk360_ImageHandle image = Blk360_Measurement_GetImageAtIndex(measurement_, imagesDone - 1);
                check("GetImageAtIndex");

                Blk360_ProcessingWorkflow_ProcessImage(processingWorkflow_, image);
                check("ProcessImage");

                if (event.imagesDone == event.imagesTotal)
                {
                    publishProgress("Downloading panorama done.");
                    break;
                }
            }
            else if (baseEvent.sender.handle == imageDownloadErrorEvents.handle)
            {
                throwEventError(baseEvent, "panorama download");
            }
            else
            {
                throw ScanError("Unexpected event during panorama download.");
            }
        }

        // --- Prepare colorizer ---
        colorizer_ = Blk360_PointCloudColorizer_New(1024);
        check("PointCloudColorizer_New");

        Blk360_ProcessingWorkflow_PointCloudColorizer_Initialize(processingWorkflow_, colorizer_, measurement_);
        check("PointCloudColorizer_Initialize");

        const Blk360_SubscriptionHandle colorizerProgress = Blk360_ProcessingWorkflow_OnPointCloudColorizerPanoramaProgress(processingWorkflow_, queue_);
        check("OnPointCloudColorizerPanoramaProgress");

        const Blk360_SubscriptionHandle colorizerError = Blk360_ProcessingWorkflow_OnPointCloudColorizerPanoramaError(processingWorkflow_, queue_);
        check("OnPointCloudColorizerPanoramaError");

        Blk360_ImageEnumeratorHandle images = Blk360_Measurement_GetPanorama(measurement_);
        check("GetPanorama");

        Blk360_ProcessingWorkflow_PointCloudColorizer_AddPanorama(processingWorkflow_, colorizer_, images);
        check("PointCloudColorizer_AddPanorama");

        while (Blk360_EventQueue_Wait(queue_, 20000))
        {
            const Blk360_Event baseEvent = Blk360_EventQueue_Pop(queue_);
            check("EventQueue_Pop (colorizer)");

            if (baseEvent.sender.handle == colorizerProgress.handle)
            {
                const auto& progressEvent = reinterpret_cast<const Blk360_PointCloudColorizerPanoramaProgressEvent&>(baseEvent);
                publishProgress("Adding images to colorizer progress " + std::to_string(progressEvent.progress) + "%");

                if (progressEvent.progress == 100)
                {
                    break;
                }
            }
            else if (baseEvent.sender.handle == colorizerError.handle)
            {
                throwEventError(baseEvent, "colorizer");
            }
            else
            {
                throw ScanError("Unexpected event during colorization setup.");
            }
        }

        Blk360_ImageEnumerator_Release(images);
        check("ImageEnumerator_Release");

        if (!Blk360_PointCloudColorizer_IsReady(colorizer_))
        {
            check("PointCloudColorizer_IsReady");
            throw ScanError("Point cloud colorizer is not ready!");
        }
        publishProgress("Colorizer ready.");

        // --- Download + process point cloud ---
        Blk360_DataManipulationWorkflow_RefreshMeasurement(dataManipulationWorkflow_, measurement_);
        check("RefreshMeasurement");

        const auto pointCloud = Blk360_Measurement_GetPointCloud(measurement_);

        const auto dataManipulationOnError = Blk360_DataManipulationWorkflow_OnError(dataManipulationWorkflow_, queue_);
        check("DataManipulationWorkflow_OnError (point cloud)");

        const auto onDownloadProgress = Blk360_DataManipulationWorkflow_OnPointCloudDownloadProgress(dataManipulationWorkflow_, queue_);
        check("OnPointCloudDownloadProgress");

        Blk360_DataManipulationWorkflow_DownloadPointCloud(dataManipulationWorkflow_, pointCloud);
        check("DownloadPointCloud");

        const auto pointCloudChunkAvailable = Blk360_ProcessingWorkflow_OnPointCloudChunkAvailable(processingWorkflow_, queue_);
        check("OnPointCloudChunkAvailable");

        const auto pointCloudProgress = Blk360_ProcessingWorkflow_OnPointCloudProcessProgress(processingWorkflow_, queue_);
        check("OnPointCloudProcessProgress");

        const auto pointCloudError = Blk360_ProcessingWorkflow_OnPointCloudProcessError(processingWorkflow_, queue_);
        check("OnPointCloudProcessError");

        Blk360_ProcessingWorkflow_ProcessPointCloud(processingWorkflow_, pointCloud);
        check("ProcessPointCloud");

        std::ofstream outFile = createOutputFile(scanId);
        if (!outFile.is_open())
        {
            throw ScanError("Could not create output file.");
        }

        while (Blk360_EventQueue_Wait(queue_, 20000))
        {
            Blk360_Event baseEvent = Blk360_EventQueue_Pop(queue_);
            check("EventQueue_Pop (point cloud)");

            if (baseEvent.sender.handle == dataManipulationOnError.handle ||
                baseEvent.sender.handle == pointCloudError.handle)
            {
                throwEventError(baseEvent, "point cloud");
            }
            else if (baseEvent.sender.handle == onDownloadProgress.handle)
            {
                const auto& progress = reinterpret_cast<const Blk360_PointCloudDownloadProgressEvent&>(baseEvent);
                const float downloadProgress = static_cast<float>(progress.downloadedSize) / progress.totalSize * 100.0f;
                publishProgress("Downloading progress: " + std::to_string(downloadProgress) + "%");
            }
            else if (baseEvent.sender.handle == pointCloudProgress.handle)
            {
                const auto& progress = reinterpret_cast<const Blk360_PointCloudProcessProgressEvent&>(baseEvent);
                if (progress.scanId != scanId)
                {
                    continue;
                }

                publishProgress("Processing progress: " + std::to_string(progress.progress) + "%");
                if (progress.progress == 100)
                {
                    break;
                }
            }
            else if (baseEvent.sender.handle == pointCloudChunkAvailable.handle)
            {
                const auto& chunk = reinterpret_cast<const Blk360_PointCloudChunkAvailableEvent&>(baseEvent);
                if (chunk.scanId != scanId)
                {
                    continue;
                }

                RCLCPP_INFO(get_logger(), "Chunk %u / %u", chunk.chunkIndex, chunk.totalChunks);

                Blk360_PointCloudColorChunkHandle colorsChunk =
                    Blk360_ProcessingWorkflow_PointCloudColorizer_Colorize(processingWorkflow_, colorizer_, chunk.handle);
                check("PointCloudColorizer_Colorize");

                const char* chunkData = Blk360_PointCloudChunk_GetData(chunk.handle);
                const char* colorData = Blk360_PointCloudColorChunk_GetData(colorsChunk);
                const std::uint64_t chunkSize = Blk360_PointCloudChunk_GetDataSizeInBytes(chunk.handle);
                const std::uint64_t colorsSize = Blk360_PointCloudColorChunk_GetDataSizeInBytes(colorsChunk);
                pointcloud::writeChunk(outFile, chunkData, chunkSize, colorData, colorsSize);

                Blk360_PointCloudColorChunk_Release(colorsChunk);
                Blk360_PointCloudChunk_Release(chunk.handle);
                check("PointCloudChunk_Release");
            }
            else
            {
                throw ScanError("Unexpected event during point cloud processing.");
            }
        }
    }

    // Reads an error event payload, logs it and throws so the worker unwinds.
    [[noreturn]] void throwEventError(const Blk360_Event& baseEvent, const char* phase)
    {
        const auto& error = reinterpret_cast<const Blk360_ErrorEvent&>(baseEvent);
        throw ScanError(std::string(phase) + " error " + std::to_string(error.errorCode) + ": " + error.message);
    }

    // ---- members -----------------------------------------------------------

    std::string device_address_;
    std::string trigger_command_;
    std::string output_dir_;
    std::string density_param_;
    std::string panorama_param_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progress_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;

    std::atomic<bool> busy_{false};
    std::thread worker_;

    Blk360_SessionHandle session_{Blk360_Handle_Null()};
    Blk360_MeasurementHandle measurement_{Blk360_Handle_Null()};
    Blk360_EventQueueHandle queue_{Blk360_Handle_Null()};
    Blk360_SystemWorkflowHandle systemWorkflow_{Blk360_Handle_Null()};
    Blk360_DataManipulationWorkflowHandle dataManipulationWorkflow_{Blk360_Handle_Null()};
    Blk360_MeasurementWorkflowHandle measurementWorkflow_{Blk360_Handle_Null()};
    Blk360_ProcessingWorkflowHandle processingWorkflow_{Blk360_Handle_Null()};
    Blk360_PointCloudColorizerHandle colorizer_{Blk360_Handle_Null()};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Blk360ScannerNode>());
    rclcpp::shutdown();
    return 0;
}

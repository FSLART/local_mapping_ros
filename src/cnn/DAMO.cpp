//
// Created by carlostojal on 01-06-2023.
//

#include "local_mapping_ros/cnn/DAMO.h"

namespace t24e::local_mapper::cnn {

    DAMO::DAMO(std::string& modelPath) {

        this->modelPath = modelPath;
        this->modelPathSet = true;

        // initialize the thread pool manager
        this->threadPool = std::make_unique<ThreadPool>(std::thread::hardware_concurrency());

    }

    void DAMO::init() {

        // verify the model path has been set
        if(!this->modelPathSet)
            throw std::runtime_error("Model path must be set before initialization!");

        // create environment
        this->env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "DAMO-YOLO");

        // create session
        Ort::SessionOptions sessionOptions;
        sessionOptions.AppendExecutionProvider_CUDA(0); // enable CUDA
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL); // enable optimizations
        this->session = std::make_unique<Ort::Session>(*this->env, this->modelPath.c_str(), sessionOptions);

        // get the input dimensions
        Ort::TypeInfo inputTypeInfo = this->session->GetInputTypeInfo(0);
        auto tensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        this->inputDims = tensorInfo.GetShape();

        // get the probabilities output dimensions
        Ort::TypeInfo probsTypeInfo = this->session->GetOutputTypeInfo(0);
        auto probsTensorInfo = probsTypeInfo.GetTensorTypeAndShapeInfo();
        this->outputProbsDims = probsTensorInfo.GetShape();

        // get the boxes output dimensions
        Ort::TypeInfo boxesTypeInfo = this->session->GetOutputTypeInfo(1);
        auto boxesTensorInfo = boxesTypeInfo.GetTensorTypeAndShapeInfo();
        this->outputBoxesDims = boxesTensorInfo.GetShape();

        std::cout << "ONNX model loaded successfully!" << std::endl;

        this->initDone = true;
    }

    std::vector<bounding_box_t> DAMO::detectCones(cv::Mat img) {

        // resize the image to inference dimensions
        cv::resize(img, img, cv::Size(DETECTOR_WIDTH, DETECTOR_HEIGHT));

        // normalize the pixel values
        cv::Mat normalizedImg;
        img.convertTo(normalizedImg, CV_32F, 2.0 / 255.0, -1.0);

        // convert HWC to CHW
        cv::Mat chwImg;
        cv::dnn::blobFromImage(normalizedImg, chwImg);

        // assign the values to a vector
        std::vector<float> inputTensorValues(DETECTOR_HEIGHT * DETECTOR_WIDTH * 3);
        inputTensorValues.assign(chwImg.begin<float>(), chwImg.end<float>());

        // create a tensor from the input values
        std::vector<Ort::Value> inputTensors;
        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
        inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, inputTensorValues.data(), inputTensorValues.size(), inputDims.data(), inputDims.size()));

        // TODO: create the output tensors
        // TODO: run the inference
        // TODO: refactor post processing code to use ONNX tensors instead of Torch tensors

        // final vector of bounding boxes
        std::vector<bounding_box_t> bounding_boxes;

        #ifdef WITH_CUDA

        // TODO: FILTERING USING CUDA (GPU)

        bounding_boxes = post_processing::Filtering::nmsIoU(classProbs, MAX_ENTROPY_THRESHOLD, MIN_SCORE_THRESHOLD,
                        IOU_THRESHOLD, bboxes);
    
        #else

        // FILTERING USING A THREAD POOL (CPU)

        bounding_boxes = post_processing::Filtering::nmsIoU(classProbs, MAX_ENTROPY_THRESHOLD, MIN_SCORE_THRESHOLD,
                        IOU_THRESHOLD, bboxes);

        #endif

        // return a vector of bounding boxes
        return bounding_boxes;
    }

    std::string DAMO::getModelPath() const {
        // verify the model path has been set
        if(!this->modelPathSet) {
            throw std::runtime_error("The model path was not set!");
        }

        return this->modelPath;
    }

    void DAMO::setModelPath(const std::string& path) {
        this->modelPathSet = true;
        this->modelPath = path;
    }

} // t24e::local_mapper::cnn

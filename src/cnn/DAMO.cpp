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

        // load the ONNX model
        try {
            std::cout << "Loading the ONNX model at " << this->modelPath << std::endl;
            this->session = this->env.CreateSession(this->modelPath.c_str(), Ort::SessionOptions{nullptr});
        } catch(const Ort::Exception& e) {
            std::cerr << "Error loading the ONNX model: " << e.what() << std::endl;
            throw std::runtime_error("Error loading the ONNX model!");

        }

        std::cout << "ONNX model loaded successfully!" << std::endl;

        this->initDone = true;
    }

    std::vector<bounding_box_t> DAMO::detectCones(cv::Mat img) {

        // resize the image to inference dimensions
        cv::Mat resizedImg;
        cv::resize(img, resizedImg, cv::Size(DETECTOR_WIDTH, DETECTOR_HEIGHT));

        // normalize the input image
        // cv::normalize(resizedImg, resizedImg, 0, 1, cv::NORM_MINMAX, CV_32F);

        // convert the opencv image to a tensor
        at::Tensor tensorImage = torch::from_blob(resizedImg.data, {1, 3, resizedImg.rows, resizedImg.cols}, at::kByte);
        // reshape to CxHxW
        // tensorImage = tensorImage.permute({0, 3, 1, 2});

        // convert to float and normalize
        tensorImage = tensorImage.to(at::kFloat) / 255.0f;

        // normalize to zero mean and unit standard deviation
        float mean = tensorImage.mean().item().toFloat(); // obtain the mean
        float std = tensorImage.std().item().toFloat(); // obtain the standard deviation
        tensorImage = (tensorImage - mean) / std; // normalize

        // std::cout << resizedImg << std::endl;

        // check the device is available
        // this->validateDevice();

        try {
            // move the tensor to the intended device
            tensorImage = tensorImage.to(this->device);
        } catch(const std::exception& e) {
            std::cerr << "Error moving the tensor to the device: " << e.what() << std::endl;
            throw std::runtime_error("Error moving the tensor to the device!");
        }

        // disable gradient tracking
        torch::NoGradGuard no_grad;

        // execute the model
        auto inference = this->torchModule.forward({tensorImage});

        auto outputs = inference.toTuple();

        /*
            tensors:
            [0]: [1, 8400, 5] -> class probabilities
            [1]: [1, 8400, 4] -> box origin and dimensions
        */

        // convert the class probabilities to tensor
        torch::Tensor classProbs = outputs->elements()[0].toTensor();

        // convert the bounding boxes to tensor
        torch::Tensor bboxes = outputs->elements()[1].toTensor();

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

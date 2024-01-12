//
// Created by carlostojal on 01-06-2023.
//

#include "local_mapping_ros/cnn/DAMO.h"

namespace t24e::local_mapper::cnn {

    DAMO::DAMO(std::string& modelPath) {

        this->modelPath = modelPath;
        this->modelPathSet = true;

        // initialize the device
        #ifdef WITH_CUDA
            this->device = torch::Device(torch::kCUDA);
            this->validateDevice();
        #else
            this->device = torch::Device(torch::kCPU);
        #endif

        // initialize the thread pool manager
        this->threadPool = std::make_unique<ThreadPool>(std::thread::hardware_concurrency());

    }

    void DAMO::validateDevice() {

        if(this->device == torch::Device(torch::kCUDA)) {
            if(!torch::cuda::is_available()) {
                std::cerr << "CUDA is not available!" << std::endl;
                throw std::runtime_error("CUDA is not available!");
            }
        }
    }

    void DAMO::init() {

        // verify the model path has been set
        if(!this->modelPathSet)
            throw std::runtime_error("Model path must be set before initialization!");

        // load the torchscript model
        try {
            std::cout << "Loading the TorchScript module at " << this->modelPath << std::endl;
            this->validateDevice();

            this->torchModule = torch::jit::load(this->modelPath, this->device);
            this->torchModule.to(this->device, torch::kFloat);
            
            this->torchModule.eval(); // set the model to evaluation mode

        } catch(const c10::Error& e) {
            std::cerr << "Error loading the TorchScript module: " << e.what() << std::endl;
            throw std::runtime_error("Error loading the TorchScript module!");
        }

        std::cout << "TorchScript module loaded successfully!" << std::endl;

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

        at::Tensor randomTensor = torch::rand({1, 3, DETECTOR_WIDTH, DETECTOR_HEIGHT});

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

        std::cout << classProbs[0].slice(1, 0, 5) << std::endl;

        // convert the bounding boxes to tensor
        torch::Tensor bboxes = outputs->elements()[1].toTensor();

        // final vector of bounding boxes
        std::vector<bounding_box_t> bounding_boxes;

        #ifdef WITH_CUDA
        
        // TODO: FILTERING USING CUDA (GPU)
        throw std::runtime_error("CUDA filtering not implemented!");

        #else

        // FILTERING USING A THREAD POOL (CPU)

        // define a mutex to access the valid samples set
        std::mutex mut;

        // define a job to calculate the entropy and find the maxima
        auto job = [classProbs, bboxes, &mut, &bounding_boxes](size_t threadIdx) {

            // get the row index
            // TODO: thread index should be reset. overflow can crash execution.
            size_t idx = threadIdx;

            // std::cout << "-----------JOB " << idx << "---------------" << std::endl;

            float max = -1.0f;
            ssize_t maxIndex = -1;

            float entropy = 0;

            // for each class of the sample
            for(ssize_t classIdx = 0; classIdx < classProbs.sizes()[2]; classIdx++) {

                float p = classProbs[0][idx][classIdx].item().toFloat();
                // std::cout << "p=" << p << std::endl;

                // check if this probability is the max of the sample
                if(p > max) {
                    max = p;
                    maxIndex = classIdx;
                }

                // construct the entropy
                entropy += p * logf(p);
            }

            entropy = -entropy;

            // std::cout << "Entropy: " << entropy << std::endl;

            /*
            // if the entropy value is elegible as a valid detection
            if(entropy <= MAX_ENTROPY_THRESHOLD) {

                // create the bounding box, extracting information from the tensors
                bounding_box_t box;
                box.box.first.first = bboxes[0][idx][0].item().toInt();
                box.box.first.second = bboxes[0][idx][1].item().toInt();

                box.box.second.first = bboxes[0][idx][2].item().toInt();
                box.box.second.second = bboxes[0][idx][3].item().toInt();

                box.label = maxIndex;

                // acquire the valid samples set mutex
                std::unique_lock<std::mutex> lk(mut);

                // add the new bounding box
                bounding_boxes.push_back(box);
            }*/
        };

        // for each sample, start a job
        for(ssize_t i = 0; i < classProbs.sizes()[1]; i++) {
            
            // enqueue the job
            this->threadPool->queueJob(job);
        }

        // start the pool
        this->threadPool->start();

        // wait for the pool to finish
        this->threadPool->join();

        // reset thread index
        this->threadPool->resetThreadIdx();

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

//
// Created by carlostojal on 12-01-2024.
//

#include <local_mapping_ros/post_processing/Filtering.h>

namespace t24e::local_mapper::post_processing {

    float Filtering::entropyRow(at::Tensor& row) {
        // calculate the entropy of the row
        float entropy = 0;
        for(int i = 0; i < row.size(0); i++) {
            float p = row[i].item<float>();
            entropy += p * std::log(p);
        }
        return -entropy;
    }

    float Filtering::maxRow(at::Tensor& row) {
        // calculate the entropy of the row
        float max = 0;
        for(int i = 0; i < row.size(0); i++) {
            float p = row[i].item<float>();
            if(p > max) {
                max = p;
            }
        }
        return max;
    }

    std::pair<at::Tensor,size_t> Filtering::filter(at::Tensor& predictions, float entThreshold, float scoreThreshold) {
        // filtered row counter
        size_t numRows = 0;

        // initialize the filtered predictions tensor with the same size as the predictions tensor filled with zeros
        at::Tensor filteredPredictions = torch::zeros_like(predictions);

        // create the thread pool
        ThreadPool pool((std::thread::hardware_concurrency()));

        // mutex to protect concurrent access to the filtered predictions tensor
        std::mutex mut;

        // iterate predictions and add jobs to the pool
        for(int i = 0; i < predictions.size(0); i++) {

            auto job = [&predictions, &filteredPredictions, &numRows, &mut, entThreshold, scoreThreshold](size_t threadIdx){

                at::Tensor row = predictions[threadIdx];

                // calculate the entropy and maximum of the row
                float entropy = entropyRow(row);
                float max = maxRow(row);

                // verify filter conditions 
                if(entropy < entThreshold && max > scoreThreshold) {
                    std::unique_lock lk(mut);
                    filteredPredictions[numRows++] = row;
                }
            };
            
            pool.queueJob(job);
        }

        // start the pool
        pool.start();

        // wait for completion
        pool.join();

        return std::make_pair(filteredPredictions, numRows);
    }

    float Filtering::calculateIoU(cnn::bounding_box_t& box1, cnn::bounding_box_t& box2) {
        
        // get coordinates of intersection rectangle
        int xA = std::max(box1.box.first.first, box2.box.first.first);
        int yA = std::max(box1.box.first.second, box2.box.first.second);
        int xB = std::min(box1.box.first.first + box1.box.second.first, box2.box.first.first + box2.box.second.first);
        int yB = std::min(box1.box.first.second + box1.box.second.second, box2.box.first.second + box2.box.second.second);


        // compute the area of intersection rectangle
        int interArea = std::max(0, xB - xA + 1) * std::max(0, yB - yA);

        // compute the area of the union
        int area1 = box1.box.second.first * box1.box.second.second;
        int area2 = box2.box.second.first * box2.box.second.second;
        int unionArea = area1 + area2 - interArea;

        // compute the intersection over union
        float iou = static_cast<float>(interArea) / static_cast<float>(unionArea);

        return iou;
    }

    std::vector<cnn::bounding_box_t> Filtering::nmsIoU(at::Tensor& predictions, float entThreshold, float scoreThreshold, float IoUThreshold,
    at::Tensor& boundingBoxesTensor) {

        std::vector<cnn::bounding_box_t> boundingBoxes;

        // filter the predictions based on the entropy of the prediction
        std::pair<at::Tensor,size_t> filtered;
        at::Tensor filteredPredictions = predictions;

        filtered = filter(predictions, entThreshold, scoreThreshold);
        filteredPredictions = filtered.first;

        // convert the tensor to a vector of bounding boxes
        for(int i = 0; i < filteredPredictions.size(0); i++) {
            at::Tensor row = filteredPredictions[i];
            float score = row[0].item<float>();
            if(score > scoreThreshold) {
                // get the bounding box
                cnn::bounding_box_t box;
                box.score = score;
                box.box.first.first = boundingBoxesTensor[i][1].item<int>();
                box.box.first.second = boundingBoxesTensor[i][2].item<int>();
                box.box.second.first = boundingBoxesTensor[i][3].item<int>();
                box.box.second.second = boundingBoxesTensor[i][4].item<int>();
                boundingBoxes.push_back(box);
            }
        }

        // sort the bounding boxes by score
        std::sort(boundingBoxes.begin(), boundingBoxes.end(), [](cnn::bounding_box_t& box1, cnn::bounding_box_t& box2) {
            return box1.score > box2.score;
        });

        ThreadPool pool(std::thread::hardware_concurrency());

        // mutex to protect concurrent access to the bounding boxes vector
        std::mutex mut;

        // perform non-maximum supression in a thread pool
        for(size_t i = 0; i < boundingBoxes.size(); i++) {
            for(size_t j = i + 1; j < boundingBoxes.size(); j++) {

                auto job = [&boundingBoxes, &i, &j, &IoUThreshold, &mut](size_t threadIdx){

                    (void)(threadIdx);

                    float iou = calculateIoU(boundingBoxes[i], boundingBoxes[j]);
                    if(iou > IoUThreshold) {
                        std::unique_lock<std::mutex> lk(mut);
                        boundingBoxes.erase(boundingBoxes.begin() + j);
                        j--;
                    }
                };

                // at each iteration, add a job to the pool
                pool.queueJob(job);
            }
        }

        // start the pool
        pool.start();

        // wait for completion
        pool.join();
        
        return boundingBoxes;
    }
};
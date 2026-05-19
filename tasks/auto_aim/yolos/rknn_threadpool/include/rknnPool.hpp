#ifndef RKNNPOOL_H
#define RKNNPOOL_H

#include "ThreadPool.hpp"
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

// rknnModel模型类, inputType模型输入类型, outputType模型输出类型
template<typename rknnModel, typename inputType, typename outputType>
class rknnPool {
private:
    int threadNum;
    std::string modelPath;

    long long id;
    std::mutex idMtx, queueMtx;
    std::unique_ptr<dpool::ThreadPool> pool;
    std::queue<std::future<outputType>> futs;
    std::vector<std::shared_ptr<rknnModel>> models;

protected:
    int getModelId();

public:
    rknnPool(const std::string modelPath, int threadNum);
    int init();
    // 模型推理/Model inference
    int put(inputType inputData);
    // 获取推理结果/Get the results of your inference
    int get(outputType& outputData);
    // 非阻塞获取推理结果/Try getting inference result without blocking
    int try_get(outputType& outputData);
    ~rknnPool();
};

template<typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::rknnPool(const std::string modelPath, int threadNum) {
    this->modelPath = modelPath;
    this->threadNum = threadNum;
    this->id        = 0;
}

template<typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::init() {
    try {
        this->pool = std::make_unique<dpool::ThreadPool>(this->threadNum);
        for (int i = 0; i < this->threadNum; i++)
            models.push_back(std::make_shared<rknnModel>(this->modelPath.c_str()));
    } catch (const std::bad_alloc& e) {
        std::cout << "Out of memory: " << e.what() << std::endl;
        return -1;
    }
    // 初始化模型/Initialize the model
    for (int i = 0, ret = 0; i < threadNum; i++) {
        ret = models[i]->init(models[0]->get_pctx(), i != 0);
        if (ret != 0)
            return ret;
    }

    return 0;
}

template<typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::getModelId() {
    std::lock_guard<std::mutex> lock(idMtx);
    int modelId = id % threadNum;
    id++;
    return modelId;
}

template<typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData) {
    std::lock_guard<std::mutex> lock(queueMtx);
    futs.push(pool->submit(&rknnModel::infer, models[this->getModelId()], inputData));
    return 0;
}

template<typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::get(outputType& outputData) {
    std::future<outputType> fut;
    {
        std::lock_guard<std::mutex> lock(queueMtx);
        if (futs.empty() == true)
            return 1;
        fut = std::move(futs.front());
        futs.pop();
    }
    outputData = fut.get();
    return 0;
}

template<typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::try_get(outputType& outputData) {
    std::future<outputType> fut;
    {
        std::lock_guard<std::mutex> lock(queueMtx);
        if (futs.empty() == true)
            return 1;
        if (futs.front().wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            return 2;
        fut = std::move(futs.front());
        futs.pop();
    }
    outputData = fut.get();
    return 0;
}

template<typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::~rknnPool() {
    while (!futs.empty()) {
        auto fut = std::move(futs.front());
        futs.pop();
        outputType temp = fut.get();
    }
}

#endif

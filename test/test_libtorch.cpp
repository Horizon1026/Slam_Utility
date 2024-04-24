#include "log_report.h"
#include "libtorch.h"

struct Net : torch::nn::Module {
    Net() :
        conv1(torch::nn::Conv2dOptions(1, 2, /*kernel_size=*/3)),
        conv2(torch::nn::Conv2dOptions(2, 4, /*kernel_size=*/3)),
        fc1(576, 128),
        fc2(128, 10) {
        register_module("conv1", conv1);
        register_module("conv2", conv2);
        register_module("conv2_drop", conv2_drop);
        register_module("fc1", fc1);
        register_module("fc2", fc2);
    }

    torch::Tensor forward(torch::Tensor x) {
        // input : 1*28*28.
        // conv1 : 28 - 3 + 1 = 26.
        x = torch::relu(conv1->forward(x));
        // input : 26*26*2.
        // conv2 : 26 - 3 + 1 = 24.
        // max_pool : 12 * 12 * 4 = 576.
        x = torch::max_pool2d(torch::relu(conv2->forward(x)), 2);
        x = torch::dropout(x, /*p=*/0.25, is_training());
        x = x.view({-1, 576});
        // w : 128 * 576.
        x = torch::relu(fc1->forward(x));
        x = torch::dropout(x, /*p=*/0.5, is_training());
        // w : 10 * 128.
        x = fc2->forward(x);
        x = torch::log_softmax(x, 1);
        return x;
    }

    torch::nn::Conv2d conv1;
    torch::nn::Conv2d conv2;
    torch::nn::Dropout conv2_drop;
    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
};

void Train(std::shared_ptr<Net> &net) {
    const int32_t batch_size = 64;
    auto data_loader = torch::data::make_data_loader(
        torch::data::datasets::MNIST("/mnt/d/My_Github/Datasets/Mnist").map(
            torch::data::transforms::Stack<>()), batch_size);

    torch::optim::SGD optimizor(net->parameters(), 0.01);

    for (int32_t epoch = 0; epoch < 10; ++epoch) {
        int32_t batch_index = 0;

        for (auto &batch : *data_loader) {
            // Reset gradients.
            optimizor.zero_grad();
            // Execute the model on the input data.
            torch::Tensor prediction = net->forward(batch.data);
            // Compute a loss value to judge the prediction of our model.
            torch::Tensor loss = torch::nll_loss(prediction, batch.target);
            // Compute gradients of the loss w.r.t. the parameters of our model.
            loss.backward();
            // Update the parameters based on the calculated gradients.
            optimizor.step();
            // Print the loss and check point.
            if (++batch_index % 100 == 0) {
                ReportInfo("[Train] epoch [" << epoch << "], batch_idx [" << batch_index << "], loss [" << loss.item<float>() << "].");
                torch::save(net, "net.pt");
            }
        }
    }
}

void Test(std::shared_ptr<Net> &net) {
    const int32_t batch_size = 64;
    auto dataset = torch::data::datasets::MNIST("/mnt/d/My_Github/Datasets/Mnist",
        torch::data::datasets::MNIST::Mode::kTest).map(
            torch::data::transforms::Stack<>());
    const auto dataset_size = dataset.size().value();
    auto data_loader = torch::data::make_data_loader(std::move(dataset), batch_size);

    float average_loss = 0.0;
    int32_t correct_cnt = 0;
    for (auto &batch : *data_loader) {
        // Execute the model on the input data.
        torch::Tensor output = net->forward(batch.data);
        // Compute a loss value to judge the output of our model.
        torch::Tensor loss = torch::nll_loss(output, batch.target, /*weight=*/{}, torch::Reduction::Sum);
        average_loss += loss.item<float>();
        // Check correction.
        torch::Tensor prediction = output.argmax(1);
        correct_cnt += prediction.eq(batch.target).sum().item<int32_t>();
    }

    average_loss /= static_cast<float>(dataset_size);
    const float corr_rate = static_cast<float>(correct_cnt) / static_cast<float>(dataset_size);

    ReportInfo("[Test] average loss [" << average_loss << "], correct rate [" << corr_rate <<
        "], correct cnt [" << correct_cnt << "/" << dataset_size << "]");
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test libtorch." RESET_COLOR);

    auto net = std::make_shared<Net>();
    Train(net);
    Test(net);

    return 0;
}

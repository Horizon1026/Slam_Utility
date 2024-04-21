#include "log_report.h"
#include "libtorch.h"

struct Net : torch::nn::Module {
    Net() {
        fc1 = register_module("fc1", torch::nn::Linear(784, 64));
        fc2 = register_module("fc2", torch::nn::Linear(64, 32));
        fc3 = register_module("fc3", torch::nn::Linear(32, 10));
    }

    torch::Tensor forward(torch::Tensor x) {
        x = torch::relu(fc1->forward(x.reshape({x.size(0), 784})));
        x = torch::dropout(x, 0.5, is_training());
        x = torch::relu(fc2->forward(x));
        x = torch::log_softmax(fc3->forward(x), 1);
        return x;
    }

    torch::nn::Linear fc1{nullptr};
    torch::nn::Linear fc2{nullptr};
    torch::nn::Linear fc3{nullptr};
};

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test libtorch." RESET_COLOR);

    auto net = std::make_shared<Net>();

    auto data_loader = torch::data::make_data_loader(
        torch::data::datasets::MNIST("/mnt/d/My_Github/Datasets/Mnist").map(
            torch::data::transforms::Stack<>()
        ), 64
    );

    torch::optim::SGD optimizor(net->parameters(), 0.01);

    for (int32_t epoch = 0; epoch < 20; ++epoch) {
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
                ReportInfo("epoch [" << epoch << "], batch_idx [" << batch_index << "], loss [" << loss.item<float>() << "].");
                torch::save(net, "net.pt");
            }
        }
    }

    return 0;
}

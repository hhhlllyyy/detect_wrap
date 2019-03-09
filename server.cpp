//
// Created by gh on 2/18/19.
//

#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>


#include "grpc/api.grpc.pb.h"
#include "rectify.h"


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using rectify::RectifyReq;
using rectify::RectifyResp;
using rectify::API;

// Logic and data behind the server's behavior.
class GreeterServiceImpl final : public API::Service {
    Status rectify(ServerContext* context, const RectifyReq* request,
                    RectifyResp* reply) override {

        std::vector<char> input;
        input.resize(request->image().size());
        memcpy(input.data(), request->image().data(), request->image().size());

        imo_image image;
        image.len = request->image().size();
        image.buf = input.data();

        imo_image dst;
        dst.len = 0;

        auto ret = imo_wrap_image_perspective(&image, &dst);
        reply->set_image(dst.buf, dst.len);

        if(ret >= 0) {
            free_image(&dst);
        }


        return Status::OK;
    }
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    GreeterServiceImpl service;

    ServerBuilder builder;
    builder.SetMaxSendMessageSize(50 * 1024 * 1024);
    builder.SetMaxReceiveMessageSize(50 * 1024 * 1024);
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char** argv) {
    RunServer();

    return 0;
}

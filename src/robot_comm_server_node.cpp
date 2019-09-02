#include <iostream>
#include <boost/network/protocol/http/server.hpp>
#include <signal.h>
#include <ros/ros.h>
#include <hospitality/FindRoute.h>

namespace http = boost::network::http;

struct http_handler
{
    findRouteClient.
}

class RobotCommServer
{
public:
    RobotCommServer(int argc, char **argv);

private:
    struct http_handler;
    typedef http::server<http_handler> http_server;

private:
    http_server server_;
    http_handler handler_;

private:
    ros::NodeHandle n;
    ros::ServiceClient find_route_client_;
};

class ServerNode
{

};

struct RobotCommServer::http_handler
{
    void operator()(server::request const &request, server::response const &response)
    {
        server::string_type ip = source(request);
        unsigned int port = request.source_port;
        std::ostringstream data;
        data << "Hello, " << ip << ':' << port << '!';
        connection->set_status(server::connection::ok);
        connection->write(data.str());
    }
};

RobotCommServer::RobotCommServer(int argc, char **argv)
    : server_(server(http_handler))
{
    ros::init(argc, argv, "/robot_comm_server_node");
    find_route_client_ = n.serviceClient<hospitality::FindRoute>("/find_route_service");
    http::server<http_handler>::options options(http_handler);
    server = http::server<http_handler>(options.address(argv[1]).port(argv[2]));
}


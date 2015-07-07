#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include <chrono>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <sts_control.h>

using boost::asio::ip::tcp;

class server
{
public:
    server(boost::asio::io_service& io_service, short port)
        : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
        socket_(io_service)
    {
        do_accept();
    }

    void send_complete(std::shared_ptr<tcp::socket> socket,
            boost::system::error_code ec)
    {
        if (ec) {
            std::cerr << "\n! writing command failed: "
                << ec << "\n";
            std::cerr << "! removing "
                << socket->remote_endpoint().address().to_string()
                << "\n";
            // std::lock_guard<std::mutex> lock(m_);
            // clients_.erase(
            //     std::find(clients_.begin(), clients_.end(), socket));
        }
    }

    template<class T>
    void send_command(T&& command)
    {
        std::lock_guard<std::mutex> lock(m_);
        for (auto iter = clients_.begin(); iter != clients_.end(); iter++) {
            auto& socket = *iter;
            boost::asio::async_write(*socket,
                    boost::asio::buffer(&command, sizeof(command)),
                    boost::asio::transfer_all(),
                    boost::bind(&server::send_complete, this,
                        socket, boost::asio::placeholders::error));
        }
    }
private:
    void read_complete(std::shared_ptr<tcp::socket> socket, std::shared_ptr<char> buffer,
            boost::system::error_code ec)
    {
        if (ec) {
            std::cerr << "\n! socket closed: "
                << ec << "\n";
            std::cerr << "! removing "
                << socket->remote_endpoint().address().to_string()
                << "\n";
            std::lock_guard<std::mutex> lock(m_);
            clients_.erase(
                std::find(clients_.begin(), clients_.end(), socket));
        } else {
            do_read(socket);
        }
    }

    void do_read(std::shared_ptr<tcp::socket> socket)
    {
        auto buffer = std::make_shared<char>();
        socket->async_read_some(boost::asio::buffer(&buffer, sizeof(*buffer)),
                boost::bind(&server::read_complete, this,
                    socket, buffer,
                    boost::asio::placeholders::error));
    }

    void accept_completed(boost::system::error_code ec)
    {
        if (!ec) {
            std::cerr << "\n! new client "
                << socket_.remote_endpoint().address().to_string()
                << "\n";
            std::lock_guard<std::mutex> lock(m_);
            clients_.push_back(
                std::make_shared<tcp::socket>(std::move(socket_)));
        }
        do_accept();
        do_read(clients_.back());
    }

    void do_accept()
    {
        acceptor_.async_accept(socket_, boost::bind(&server::accept_completed,
                    this, boost::asio::placeholders::error));
    }

    tcp::acceptor acceptor_;
    tcp::socket socket_;
    std::mutex m_;
    std::vector<std::shared_ptr<tcp::socket>> clients_;
};

int main(int argc, char* argv[])
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: sts_control <port>\n";
            return 1;
        }

        boost::asio::io_service io_service;

        server s(io_service, std::atoi(argv[1]));
        std::thread t(boost::bind(&boost::asio::io_service::run,
                    boost::ref(io_service)));

        bool started = false;
        for (;;) {

            if (!started) {
                std::cout << "#start(sample period in ms)> ";
            } else {
                std::cout << "#dump(filename)> ";
            }


            std::string line;
            std::getline(std::cin, line);
            std::stringstream ss(line);
            if (started) {
                std::string filename;
                ss >> filename;
                if (filename.empty()) {
                    std::cout << "no filename specifed\n";
                    continue;
                }
                s.send_command(DumpCommand(filename));
                std::cout << "dumping collected stack traces...\n";
                started = false;
            } else {
                std::uint32_t period_ms = 0;
                ss >> period_ms;
                if (period_ms == 0) {
                    std::cout << "sample period invalid or not specified\n";
                    continue;
                }
                s.send_command(
                        StartCommand{std::chrono::milliseconds(period_ms)});
                std::cout << "start sampling stack traces every "
                    << period_ms << "ms...\n";
                started = true;
            }
        }
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}

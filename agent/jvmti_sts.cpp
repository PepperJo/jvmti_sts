#include <thread>
#include <chrono>
#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include <sys/types.h>
#include <unistd.h>

#include <jvmti.h>

#include <sts_control.h>

using boost::asio::ip::tcp;
static std::shared_ptr<tcp::socket> client_socket;

static const std::uint32_t max_frame_count = 32;
static volatile bool end = false;
static std::thread sample_thread;

struct ThreadStackInfos {
    jvmtiStackInfo *stack_info;
    jint n_threads;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_point;
};
typedef std::vector<ThreadStackInfos> ThreadStackInfosHistory;

static void do_dump(jvmtiEnv* jvm_env, const ThreadStackInfosHistory& history,
        const std::string& filename)
{
    using namespace std::chrono;

    pid_t pid = getpid();
    std::ofstream file(filename + "_" + std::to_string(pid));

    file << pid << "\n";
    for (auto& event : history) {
        jvmtiError err;
        auto tse = event.time_point.time_since_epoch();
        file << duration_cast<milliseconds>(tse).count() << "\n";
        for (int ti = 0; ti < event.n_threads; ti++) {
            auto& si = event.stack_info[ti];

            jvmtiThreadInfo thread_info;
            err = jvm_env->GetThreadInfo(si.thread, &thread_info);
            if (err != JVMTI_ERROR_NONE) {
                std::cerr << "Unable to get thread info (" << err << ")\n";
                return;
            }
            file << "#" << ti << " ";
            if (thread_info.name) {
                file << thread_info.name;
            }

            file << " (" << si.state << ")\n";
            for (int fi = 0; fi < si.frame_count; fi++) {
                auto& frame = si.frame_buffer[fi];
                char* name;
                char* signature;
                char* generic;
                err = jvm_env->GetMethodName(frame.method, &name,
                        &signature, &generic);
                if (err != JVMTI_ERROR_NONE) {
                    std::cerr << "Unable to get method name (" << err << ")\n";
                    return;
                }

                file << "\t#" << fi << " ";
                if (name) {
                    file << name;
                }
                if (generic) {
                    file << generic;
                }
                if (signature) {
                    file << signature;
                }
                file << " : " << frame.location << "\n";
            }
        }
        file << std::endl;

        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(event.stack_info));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return;
        }
    }
}

static void sample_stacktrace(JavaVM* jvm, jvmtiEnv* jvm_env)
{
    JNIEnv* penv;
    jint ret = jvm->AttachCurrentThreadAsDaemon(reinterpret_cast<void**>(&penv),
            NULL);
    if (ret != JNI_OK) {
        std::cerr << "Unable to attach current thread (" << ret << ")\n";
        return;
    }

    while (!end) {
        boost::system::error_code asio_error;
        client_socket->non_blocking(false, asio_error);
        if (asio_error) {
            std::cerr << "unable to set socket to blocking\n";
            goto detach;
        }

        char buffer[max_command_size];
        boost::asio::read(*client_socket, boost::asio::buffer(buffer,
                    sizeof(StartCommand)), asio_error);
        if (asio_error) {
            std::cerr << "unable to read from socket (" << asio_error << "\n";
            goto detach;
        }
        StartCommand* start_cmd = reinterpret_cast<StartCommand*>(buffer);
        if (start_cmd->type != CommandType::START) {
            std::cerr << "wrong command received -> expected start\n";
            goto detach;
        }
        std::chrono::milliseconds period(start_cmd->period_ms);

        client_socket->non_blocking(true, asio_error);
        if (asio_error) {
            std::cerr << "unable to set socket to non blocking\n";
            goto detach;
        }

        ThreadStackInfosHistory history;
        std::string filename;
        while (!end) {
            /* take sample */
            history.push_back({});
            auto& event = history.back();
            event.time_point = std::chrono::high_resolution_clock::now();
            jint n_threads;
            jvmtiError err = jvm_env->GetAllStackTraces(max_frame_count,
                    &event.stack_info, &event.n_threads);
            if (err != JVMTI_ERROR_NONE) {
                std::cerr << "Unable to get all stack traces (" << err << ")\n";
                continue;
            }

            /* sleep for some time */
            std::this_thread::sleep_for(period);

            /* check if control wants us to dump */
            boost::asio::read(*client_socket, boost::asio::buffer(buffer,
                        sizeof(DumpCommand)), asio_error);
            if (asio_error && asio_error != boost::asio::error::would_block) {
                std::cerr << "Unable to read from socket ("
                    << asio_error << "\n";
                goto detach;
            } else if (!asio_error) {
                DumpCommand* dump_cmd = reinterpret_cast<DumpCommand*>(buffer);
                if (dump_cmd->type != CommandType::DUMP) {
                    std::cerr << "wrong command received -> expected dump\n";
                    goto detach;
                }
                filename = dump_cmd->filename;
                /* dump */
                break;
            }
        }
        do_dump(jvm_env, history, filename);
    }
detach:
    jvm->DetachCurrentThread();
}

static void JNICALL cbVMInit(jvmtiEnv* jvmti, JNIEnv* env, jthread thread)
{
    JavaVM* jvm;
    jint ret = env->GetJavaVM(&jvm);
    if (ret != JNI_OK) {
        std::cerr << "Unable to get java vm (" << ret << ")\n";
        return;
    }

    sample_thread = std::thread(sample_stacktrace, jvm, jvmti);
}

static void JNICALL cbVMDeath(jvmtiEnv* jvmti, JNIEnv* env)
{
    end = true;
    /* wait until dumping is finished */
    sample_thread.join();
}

JNIEXPORT jint JNICALL Agent_OnLoad(JavaVM* jvm, char* options, void* reserved)
{
    jvmtiEnv* jvm_env = nullptr;
    jint ret = jvm->GetEnv(reinterpret_cast<void**>(&jvm_env),
            JVMTI_VERSION_1_2);
    if (ret != JNI_OK) {
        std::cerr << "Unable to get env (" << ret << ")\n";
        return JNI_ERR;
    }

    jvmtiEventCallbacks callbacks = {};
    callbacks.VMInit = &cbVMInit;
    callbacks.VMDeath = &cbVMDeath;
    jvmtiError jvmti_err = jvm_env->SetEventCallbacks(&callbacks,
            sizeof(callbacks));
    if (jvmti_err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to set event callback (" << jvmti_err << ")\n";
        return JNI_ERR;
    }
    const jvmtiEvent events[] = {JVMTI_EVENT_VM_INIT, JVMTI_EVENT_VM_DEATH};
    for (auto event : events) {
        jvmti_err = jvm_env->SetEventNotificationMode(JVMTI_ENABLE,
                event, nullptr);
        if (jvmti_err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to set event notification mode enable ("
                << jvmti_err << ")\n";
            return JNI_ERR;
        }
    }

    boost::asio::io_service io_service;
    client_socket = std::make_shared<tcp::socket>(io_service);
    tcp::resolver resolver(io_service);

    std::vector<std::string> str_options;
    boost::split(str_options, options, boost::is_any_of(","));
    if (str_options.size() != 2) {
        std::cerr << "invalid options (usage: <hostname>,<port>)\n";
        return JNI_ERR;
    }
    boost::system::error_code asio_error;
    boost::asio::connect(*client_socket,
            resolver.resolve({str_options[0], str_options[1]}),
            asio_error);
    if (asio_error) {
        std::cerr << "unable to connect to " << str_options[0] << ":"
            << str_options[1] << "\n";
        return JNI_ERR;
    }

    return JNI_OK;
}

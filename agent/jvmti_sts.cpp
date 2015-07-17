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

#define INDENT(x) std::string(x, ' ')

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

static jvmtiError print_class(jvmtiEnv* jvm_env, jclass klass,
        std::ostream& out, int indent)
{
    indent += 1;
    char* signature; char* generic;
    jvmtiError err = jvm_env->GetClassSignature(klass, &signature, &generic);
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to get class signature ("
            << err << ")\n";
        return err;
    }

    if (signature) {
        out << INDENT(indent) << "'CLASS.signature' : '''" << signature << "''', \n";
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(signature));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return err;
        }
    }

    if (generic) {
        out << INDENT(indent) << " 'CLASS.generic' : '''" << generic <<  "''', \n";
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(generic));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return err;
        }
    }
    return JVMTI_ERROR_NONE;
}

static jvmtiError print_method(jvmtiEnv* jvm_env, jvmtiFrameInfo& frame,
        std::ostream& out, int indent)
{
    indent += 1;
    char* name; char* signature; char* generic;
    jvmtiError err = jvm_env->GetMethodName(frame.method, &name,
            &signature, &generic);
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to get method name (" << err << ")\n";
        return err;
    }

    if (name) {
        out << INDENT(indent) << "'METHOD.name' : '''" << name <<  "''', \n";
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(name));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return err;
        }
    }
    if (generic) {
        out << INDENT(indent) << "'METHOD.generic' : '''" << generic <<  "''', \n";
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(generic));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return err;
        }
    }
    if (signature) {
        out << INDENT(indent) << "'METHOD.signature' : '''" << signature  << "''', \n";
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(signature));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return err;
        }
    }

    return JVMTI_ERROR_NONE;
}

static jvmtiError print_source(jvmtiEnv* jvm_env, jclass klass,
        jmethodID method, jlocation location, std::ostream& out, int indent)
{
    indent += 1;
    char* name;
    jvmtiError err = jvm_env->GetSourceFileName(klass, &name);
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to get source file name (" << err << ")\n";
        return err;
    }
    err = jvm_env->Deallocate(
            reinterpret_cast<unsigned char*>(name));
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to Deallocate (" << err << ")\n";
        return err;
    }

    out << INDENT(indent) << "'SOURCE.name' : '''" << name << "''', \n";

    jint entries;
    jvmtiLineNumberEntry* table;
    err = jvm_env->GetLineNumberTable(method, &entries, &table);
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to get line number table (" << err << ")\n";
        return err;
    }

    jint i;
    for (i = 0; i < entries; i++) {
        if (table[i].start_location > location) {
            break;
        }
    }
    if (i == 0) {
        out << INDENT(indent) << "'SOURCE.linenumber' : 'None' \n";
    } else {
        out << INDENT(indent) << "'SOURCE.linenumber' : " << table[i - 1].line_number << "\n";
    }
    err = jvm_env->Deallocate(
            reinterpret_cast<unsigned char*>(table));
    if (err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to Deallocate (" << err << ")\n";
        return err;
    }

    return JVMTI_ERROR_NONE;
}

static void do_dump(jvmtiEnv* jvm_env, const ThreadStackInfosHistory& history,
        const std::string& filename)
{
    using namespace std::chrono;

    pid_t pid = getpid();
    char hostname[256];
    int ret = gethostname(hostname, sizeof(hostname));
    if (ret) {
        std::cerr << "Unable to get hostname (" << ret << ")\n";
        return;
    }
    std::ofstream file(filename + "_" + hostname + "_" +
            std::to_string(static_cast<long long>(pid)));

    int indent = 0;
    file << INDENT(indent) << "{ ";
    indent = 1;
    file << INDENT(indent) << "'PID' : "<< pid << ",\n";
    file << INDENT(indent) << "'SAMPLES' : [ " << "\n";
    for (auto iter = history.begin(); iter != history.end(); iter++) {
        indent = 2;
        file << INDENT(indent) << "{";

        auto& event = *iter;
        jvmtiError err;
        auto tse = event.time_point.time_since_epoch();

        indent = 3;
        file << INDENT(indent) << "'SYSTIME' : " << duration_cast<milliseconds>(tse).count() << ",\n";
        file << INDENT(indent) << "'THREADS' : [ " << "\n";
        for (int ti = 0; ti < event.n_threads; ti++) {
            indent = 4;
            file << INDENT(indent) << " { \n";
            indent = 5;
            auto& si = event.stack_info[ti];
            jvmtiThreadInfo thread_info;
            err = jvm_env->GetThreadInfo(si.thread, &thread_info);
            if (err != JVMTI_ERROR_NONE) {
                std::cerr << "Unable to get thread info (" << err << ")\n";
                return;
            }
            file << INDENT(indent) << "'ID' : " << ti << ",\n";
            if (thread_info.name) {
                file << INDENT(indent) << "'ThreadName' : '''" << thread_info.name << "''' ,\n";
            }

            file << INDENT(indent) << "'ThreadState' : " << si.state << ",\n";
            file << INDENT(indent) << "'FRAMES' : { " << "\n";
            indent = 6;
            for (int fi = 0; fi < si.frame_count; fi++) {
                indent = 7;
                file << INDENT(indent) << fi << " : { \n";
//                file << "'NUMBER' : " << fi << ", \n";
                auto& frame = si.frame_buffer[fi];

                jclass klass;
                err = jvm_env->GetMethodDeclaringClass(frame.method, &klass);
                if (err != JVMTI_ERROR_NONE) {
                    std::cerr << "Unable to get method declaring class ("
                        << err << ")\n";
                    return;
                }
                err = print_class(jvm_env, klass, file, indent);
                if (err != JVMTI_ERROR_NONE) {
                    std::cerr << "Unable to print class (" << err << ")\n";
                    return;
                }
                err = print_method(jvm_env, frame, file, indent);
                if (err != JVMTI_ERROR_NONE) {
                    std::cerr << "Unable to print class (" << err << ")\n";
                    return;
                }
                if (frame.location == -1) {
                    file << INDENT(indent) << " 'Location' : None" << ", \n";
//                    file << "NATIVE";
                } else {
                    err = print_source(jvm_env, klass, frame.method,
                            frame.location, file, indent);
                    if (err != JVMTI_ERROR_NONE) {
                        std::cerr << "Unable to print class (" << err << ")\n";
                        return;
                    }
                }
                file << INDENT(indent) << " }, \n";
            } /* end frames */
            indent = 5;
            file << INDENT(indent) << "}" << "\n";
            indent = 4;
            file << INDENT(indent) << "}, " << "\n";
        } /* end threads */
        indent = 3;
        file << INDENT(indent) << "]" << "\n";
        file << std::endl;
        err = jvm_env->Deallocate(
                reinterpret_cast<unsigned char*>(event.stack_info));
        if (err != JVMTI_ERROR_NONE) {
            std::cerr << "Unable to Deallocate (" << err << ")\n";
            return;
        }
        indent = 2;
        file << INDENT(indent) << "}, " << "\n";
    } /* end samples */
    indent = 1;
    file << INDENT(indent) << "]" << "\n";
    indent = 0;
    file << INDENT(indent) << "} " << "\n";
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

        boost::asio::socket_base::non_blocking_io command(false);
        client_socket->io_control(command);

        char buffer[max_command_size];
        boost::asio::read(*client_socket, boost::asio::buffer(buffer,
                    sizeof(StartCommand)), boost::asio::transfer_all(), asio_error);
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

        command = true;
        client_socket->io_control(command);

        ThreadStackInfosHistory history;
        std::string filename;
        while (!end) {
            /* take sample */
            history.push_back(ThreadStackInfos());
            auto& event = history.back();
            event.time_point = std::chrono::high_resolution_clock::now();
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
                        sizeof(DumpCommand)), boost::asio::transfer_all(),
                    asio_error);
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
    jvmtiEnv* jvm_env = NULL;
    jint ret = jvm->GetEnv(reinterpret_cast<void**>(&jvm_env),
            JVMTI_VERSION_1_2);
    if (ret != JNI_OK) {
        std::cerr << "Unable to get env (" << ret << ")\n";
        return JNI_ERR;
    }

    auto cap = new jvmtiCapabilities{0};
    cap->can_get_source_file_name = 1;
    cap->can_get_line_numbers = 1;
    jvmtiError jvmti_err = jvm_env->AddCapabilities(cap);
    if (jvmti_err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to add capability (" << jvmti_err << ")\n";
        return JNI_ERR;
    }

    jvmtiEventCallbacks callbacks = {};
    callbacks.VMInit = &cbVMInit;
    callbacks.VMDeath = &cbVMDeath;
    jvmti_err = jvm_env->SetEventCallbacks(&callbacks, sizeof(callbacks));
    if (jvmti_err != JVMTI_ERROR_NONE) {
        std::cerr << "Unable to set event callback (" << jvmti_err << ")\n";
        return JNI_ERR;
    }
    const jvmtiEvent events[] = {JVMTI_EVENT_VM_INIT, JVMTI_EVENT_VM_DEATH};
    for (std::uint32_t i = 0; i < sizeof(events)/sizeof(*events); i++) {
        auto& event = events[i];
        jvmti_err = jvm_env->SetEventNotificationMode(JVMTI_ENABLE,
                event, NULL);
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
    client_socket->connect(*resolver.resolve({str_options[0], str_options[1]}),
            asio_error);
    if (asio_error) {
        std::cerr << "unable to connect to " << str_options[0] << ":"
            << str_options[1] << "\n";
        return JNI_ERR;
    }

    return JNI_OK;
}

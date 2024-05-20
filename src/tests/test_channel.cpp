#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "constellations/gps/gps_l1ca_channel.hpp"
#include "core/param/structures.hpp"
#include "file_operations/binary_file.hpp"
#include "file_operations/yaml_parser.hpp"

void data_file_reader(sturdivant_sdr::SdrConf conf, std::vector<int8_t>* file_stream,
                      std::vector<std::complex<double>>* data_stream, bool* repeat,
                      std::atomic<int>* threads_done, int num_threads, std::shared_mutex* m,
                      std::condition_variable_any* cv) {
    // std::cout << "here" << std::endl;
    (*repeat) = true;
    int samples_to_read = static_cast<int>(conf.sigconf.sampling_freq * 0.02);
    // file_stream->resize(samples_to_read);
    // data_stream->resize(samples_to_read);
    std::cout << "samples_to_read = " << data_stream->size() << std::endl;

    // open file
    file_operations::BinaryFile<int8_t> fid(
        "/Users/daniel/devel/signal_data/class_ifen_8bit_20e6_if_5000445.88565834.bin", 'r');
    fid.fseek(samples_to_read * conf.fileconf.ms_to_skip);

    for (uint32_t i = 0; i < conf.fileconf.ms_to_process; i += 20) {
        // wait for all threads to finish
        while ((*threads_done) < num_threads) {
            // std::cout << "threads_done = " << (*threads_done) << std::endl;
            std::this_thread::yield();  // tells scheduler to prioritize other threads
        }
        // std::cout << "threads_done = " << (*threads_done) << std::endl;

        // lock all the threads to input new data
        m->lock();
        // std::cout << "data_file_reader locked" << std::endl;

        // reset condition to incomplete
        (*threads_done) = 0;

        // read new data
        if (fid.fread(*file_stream, samples_to_read)) {
            for (int jj = 0; jj < samples_to_read; jj++) {
                data_stream->at(jj) =
                    std::complex<double>(static_cast<double>(file_stream->at(jj)), 0.0);
            }
        } else {
            std::cout << "data_file_reader failed!!!!!" << std::endl << std::endl;
        }
        // std::cout << "data_file_reader done" << std::endl;

        // unlock memory to other threads and notify completion
        m->unlock();
        cv->notify_all();
    }
    (*repeat) = false;
}

//! ================================================================================================

void run_channel(sturdivant_sdr::SdrConf conf, uint8_t* prn_id, uint32_t* channel_id,
                 std::vector<std::complex<double>>* data_stream, std::atomic<int>* threads_done,
                 bool* repeat, std::shared_mutex* m, std::condition_variable_any* cv) {
    // create channel
    sturdivant_sdr::GpsL1caChannel ch(conf, *prn_id, *channel_id);

    // fid1.fseek(20000 * conf.fileconf.ms_to_skip);
    // ch.set_data_stream_callback(&data_stream_callback);
    ch.set_input_data_vector(data_stream);
    ch.set_mutex_and_conditions(threads_done, repeat, m, cv);
    ch.run();
}

int main() {
    // load from yaml file
    file_operations::YamlParser yp("config/ifen_recording.yaml");
    sturdivant_sdr::SdrConf conf;

    // signal-file config
    conf.fileconf.filename = yp.GetVariable<std::string>("signal_source.filename");
    // conf.fileconf.active_signals =
    //     yp.GetVariable<std::vector<std::string>>("signal_source.active_signals");
    conf.fileconf.sample_type = yp.GetVariable<std::string>("signal_source.sample_type");
    conf.fileconf.ms_to_skip = yp.GetVariable<uint16_t>("signal_source.ms_to_skip");
    conf.fileconf.ms_to_process = yp.GetVariable<uint32_t>("signal_source.ms_to_process");
    conf.sigconf.sampling_freq = yp.GetVariable<double>("signal_source.sampling_freq_hz");
    conf.sigconf.mix_freq = yp.GetVariable<double>("signal_source.mix_freq_hz");

    // acquisition config
    conf.acqconf.threshold = yp.GetVariable<double>("acquisition.threshold");
    conf.acqconf.doppler_step_hz = yp.GetVariable<double>("acquisition.doppler_step_hz");
    conf.acqconf.doppler_max_hz = yp.GetVariable<double>("acquisition.doppler_max_hz");
    conf.acqconf.coherent_integration_time_ms =
        yp.GetVariable<uint8_t>("acquisition.coherent_integration_time_ms");
    conf.acqconf.dump = yp.GetVariable<bool>("acquisition.dump");
    conf.acqconf.dump_filename = yp.GetVariable<std::string>("acquisition.dump_filename");
    if (!yp.GetVariable<std::string>("acquisition.implementation").compare("pcps_fine")) {
        conf.acqconf.do_fine_search = true;
    } else {
        conf.acqconf.do_fine_search = false;
    }

    // tracking config
    conf.trkconf.fll_bw_hz = yp.GetVariable<double>("tracking.fll_bw_hz");
    conf.trkconf.pll_bw_hz = yp.GetVariable<double>("tracking.pll_bw_hz");
    conf.trkconf.dll_bw_hz = yp.GetVariable<double>("tracking.dll_bw_hz");
    conf.trkconf.correlator_spacing_chips =
        yp.GetVariable<double>("tracking.correlator_spacing_chips");
    conf.trkconf.min_pull_in_time_ms = yp.GetVariable<uint16_t>("tracking.min_pull_in_time_ms");
    conf.trkconf.dump = yp.GetVariable<bool>("tracking.dump");
    conf.trkconf.dump_filename = yp.GetVariable<std::string>("tracking.dump_filename");
    if (!yp.GetVariable<std::string>("tracking.implementation").compare("scalar_kalman_filter")) {
        conf.trkconf.do_kalman_filter = true;
    } else {
        conf.trkconf.do_kalman_filter = false;
    }

    // observables config
    conf.obsconf.cn0_estimator = yp.GetVariable<std::string>("observables.cn0_estimator");
    conf.obsconf.lock_detector_integration_time_ms =
        yp.GetVariable<uint16_t>("observables.lock_detector_integration_time_ms");
    conf.obsconf.dump = yp.GetVariable<bool>("observables.dump");
    conf.obsconf.dump_filename = yp.GetVariable<std::string>("observables.dump_filename");

    // pvt config
    conf.pvtconf.update_rate_ms = yp.GetVariable<uint16_t>("pvt.update_rate_ms");
    conf.pvtconf.display_rate_ms = yp.GetVariable<uint16_t>("pvt.display_rate_ms");
    conf.pvtconf.iono_model = yp.GetVariable<std::string>("pvt.iono_model");
    conf.pvtconf.trop_model = yp.GetVariable<std::string>("pvt.trop_model");
    conf.pvtconf.dump = yp.GetVariable<bool>("pvt.dump");
    conf.pvtconf.dump_filename = yp.GetVariable<std::string>("pvt.dump_filename");
    if (!yp.GetVariable<std::string>("pvt.implementation").compare("gnss_vector_track")) {
        conf.pvtconf.do_vector_track = true;
    } else {
        conf.pvtconf.do_vector_track = false;
    }

    //* ############################################################################################
    auto time1 = std::chrono::high_resolution_clock::now();

    // initialize mutex variables
    int samples_to_read = static_cast<int>(conf.sigconf.sampling_freq * 0.02);
    int num_threads = 5;
    bool repeat = true;
    std::vector<int8_t> file_stream(samples_to_read);
    std::vector<std::complex<double>> data_stream(samples_to_read);
    // bool threads_done[num_threads];  // = {true, true, true, true, true};
    // std::vector<bool> threads_done(num_threads, false);
    std::atomic<int> threads_done = num_threads;
    std::shared_mutex m;
    std::condition_variable_any cv;

    // create file reader
    std::thread reader(&data_file_reader, conf, &file_stream, &data_stream, &repeat, &threads_done,
                       num_threads, &m, &cv);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // create channels
    uint32_t channel_ids[5] = {1u, 2u, 3u, 4u, 5u};
    uint8_t prn_ids[5] = {1u, 7u, 14u, 21u, 30u};
    std::vector<std::thread> channels;
    // std::vector<sturdivant_sdr::GpsL1caChannel> channels(num_threads);
    for (int i = 0; i < num_threads; i++) {
        channels.push_back(std::thread(&run_channel, conf, prn_ids + i, channel_ids + i,
                                       &data_stream, &threads_done, &repeat, &m, &cv));
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // sturdivant_sdr::GpsL1caChannel ch(conf, prn_ids[i], channel_ids[i]);
        // channels[i].init(conf, prn_ids[i], channel_ids[i]);
        // channels[i].set_input_data_vector(&data_stream);
        // channels[i].set_mutex_and_conditions(threads_done + i, &repeat, &m, &cv);
        // channels[i].run();
        // channels.emplace_back(ch);
    }

    // join threads when finished
    reader.join();
    for (std::thread& c : channels) {
        c.join();
    }

    auto time2 = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1);
    std::cout << "Tracking Time over " << conf.fileconf.ms_to_process << " ms: " << ms.count()
              << " ms" << std::endl;

    return 0;
}
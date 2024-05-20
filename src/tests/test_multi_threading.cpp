#include <condition_variable>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <thread>

void publisher(double* num, int n_th, bool* repeat, bool th_done[], std::shared_mutex* m,
               std::condition_variable_any* cv) {
    (*repeat) = true;

    for (int i = 0; i < 10; i++) {
        // wait for all threads to finish
        // while (!std::all_of(th_done->begin(), th_done->end(), [](bool x) { return x; })) {
        while (!std::all_of(th_done, th_done + n_th, [](bool x) { return x; })) {
            std::this_thread::yield();  // tells scheduler to prioritize other threads
        }

        // reset thread status
        m->lock();
        for (int i = 0; i < n_th; i++) {
            // th_done->at(i) = false;
            th_done[i] = false;
        }

        // update num
        (*num) += 1.0;
        std::cout << std::endl;
        std::cout << "[\u001b[32minfo\u001b[37m] Number is: " << *num << std::endl;

        // m->unlock();  // unlock memory to other threads
        m->unlock();
        cv->notify_all();
    }
    (*repeat) = false;  // end
}

void listener(int id, double* num, bool* repeat, bool* th_done, std::shared_mutex* m,
              std::condition_variable_any* cv) {
    std::cout << "th_done[" << id << "] = " << (*th_done) << std::endl;
    while (*repeat) {
        // wait for restart
        std::shared_lock<std::shared_mutex> lk(*m);
        // cv->wait(lk, [&]() { return (!th_done->at(id)); });
        cv->wait(lk, [th_done]() { return (!(*th_done)); });

        // print function result and update
        std::cout << "[\u001b[32minfo\u001b[37m] Listener " << id << ": " << *num << " + " << id
                  << " = " << *num + id << std::endl;
        // th_done->at(id) = true;
        (*th_done) = true;
    }
}

int main() {
    auto time1 = std::chrono::high_resolution_clock::now();
    std::cout << std::boolalpha;

    // create shared variables
    const int n_th = 10;
    double num = 0.0;
    bool repeat = true;
    // std::vector<bool> th_done(n_th, false);
    bool th_done[n_th] = {false};
    std::shared_mutex m;
    std::condition_variable_any cv;

    // initialize threads
    std::vector<std::thread> list;
    for (int i = 0; i < n_th; i++) {
        list.emplace_back(std::thread(&listener, i, &num, &repeat, th_done + i, &m, &cv));
    }
    std::thread pubt(&publisher, &num, n_th, &repeat, th_done, &m, &cv);

    // join threads when finished
    for (std::thread& t : list) {
        t.join();
    }
    pubt.join();

    // return time taken
    auto time2 = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1);
    std::cout << "Time: " << ms.count() << " ms" << std::endl;

    return 0;
}
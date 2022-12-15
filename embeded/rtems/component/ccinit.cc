/*
 * Copyright 2022 wtcat
 */
#include <memory>
#include <algorithm>

#include "basework/ccinit.h"
#include "basework/cinit.h"


namespace base {

cc_initializer::~cc_initializer() {
    clear();
}

cc_initializer* cc_initializer::get_instance() {
    static cc_initializer initializer;
    return &initializer;
}

cc_constructor* cc_initializer::add_constructor(cc_constructor* c) {
    container_.push_back(c);
    return c;
}

int cc_initializer::run(int first_order, int last_order) {
    for (auto iter : container_) {
        int order = iter->major_order();
        if (order > last_order)
            break;
        if (order >= first_order) {
            int ret = iter->run();
            if (ret)
                return ret;
        }
    }
    return 0;
}

cc_initializer* cc_initializer::sort() {
    std::sort(container_.begin(), container_.end(),
        [](cc_constructor* a, cc_constructor* b) {
            return a->order() < b->order();
        }
    );
    sorted_ = true;
    return this;
}

void cc_initializer::clear() {
    for (auto iter : container_)
        delete iter;
}

int cc_initializer_run(int first_order, int last_order) {
    cc_initializer *p = cc_initializer::get_instance();
    if (!p->is_sorted()) 
        p->sort();
    return p->run(first_order, last_order);
}

int cc_initializer_run() {
    return cc_initializer_run(0, 0xFFFF);
}

} //namespace base

//C-API
extern "C" {

int c_initializer_run() {
    return base::cc_initializer_run();
}

int c_initializer_run_order(int major_order) {
    return base::cc_initializer_run(major_order, major_order);
}

void c_initializer_destroy() {
    base::cc_initializer::get_instance()->clear();
}

} //C-API

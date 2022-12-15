
/*
 * Copyright 2022 wtcat
 */

#ifndef BASE_INITIALIZER_H_
#define BASE_INITIALIZER_H_

#include <vector>
#include "basework/cinit.h"

namespace base {

class cc_constructor {
public:
    cc_constructor(int level) : order_(level) {}
    virtual ~cc_constructor() {}
    virtual int run() = 0;
    int order() const { return order_; }
    int major_order() const {return (order_ >> 16) & 0xFFFF; }
protected:
    unsigned int order_;
};

class cc_initializer {
public:
    enum {
        kFirst = 10,
        kMiddle = 50,
        kLast = 99
    };
    
    ~cc_initializer();
    cc_initializer(const cc_initializer&) = delete;
    cc_initializer(cc_initializer &&) = delete;
    cc_initializer& operator=(const cc_initializer&) = delete;
    static cc_initializer* get_instance();
    cc_constructor* add_constructor(cc_constructor* c);
    int run(int first_order, int last_order);
    cc_initializer* sort();
    bool is_sorted() const { return sorted_; }
    void clear();

private:
    cc_initializer(): sorted_(false) {}
private:
    std::vector<cc_constructor*> container_;
    bool sorted_;
};

int cc_initializer_run(int first_order, int last_order);
int cc_initializer_run();

} //namespace base

#ifndef _CONTACT
#define _CONTACT(_a, _b) _a ## _b
#endif

#define CC_INIT(_name, _major, _minor) \
    class _CONTACT(_cc_contructor_, _name) : public base::cc_constructor { \
    public:                                                       \
        _CONTACT(_cc_contructor_, _name)(int order) : base::cc_constructor(order) {}           \
        ~_CONTACT(_cc_contructor_, _name)() = default;   \
        int run() override; \
    private:                    \
        static base::cc_constructor* const construct_;  \
    };  \
    base::cc_constructor* const _CONTACT(_cc_contructor_, _name)::construct_ = \
        base::cc_initializer::get_instance()->add_constructor( \
        new _CONTACT(_cc_contructor_, _name)(((_major) << 16)|(_minor))); \
    int _CONTACT(_cc_contructor_, _name)::run()

#endif /* BASE_INITIALIZER_H_ */

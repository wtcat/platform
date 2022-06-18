extern "C" int flsl(long i);
#include <rtems.h>
#include <rtems/bspIo.h>
#include "gtest/gtest.h"
#include "bsp/timlib.h"

#define DEV_NAME "timer2"


class cc_timlib_test : public testing::Test {
public:
    void SetUp() override {
        dev_ = timlib_open(DEV_NAME);
        ASSERT_TRUE(dev_ != nullptr);
        t_ = (struct timlib_priv *)dev_->priv;
        ASSERT_TRUE(t_ != nullptr);
        ASSERT_TRUE(t_->state == 1);
    }
    void TearDown() override {
        ASSERT_TRUE(dev_ != nullptr);
        ASSERT_TRUE(t_ != nullptr);
        ASSERT_TRUE(t_->state == 1);
        timlib_close(dev_);
        ASSERT_TRUE(t_->state == 0);
    }

protected:
    struct drvmgr_dev *dev_;
    struct timlib_priv *t_;
};

extern "C"{
static volatile int isr_counter;
static void test_timer_isr(void *arg) {
    struct drvmgr_dev *dev = (struct drvmgr_dev *)arg;
    printk("isr_counter = %d\n", isr_counter);
    timlib_irq_ack(dev);
    isr_counter++;
    if (isr_counter == 3)
        timlib_stop(dev);
}

} //extern "C"

TEST_F(cc_timlib_test, open_close) {
    struct drvmgr_dev *dev;
    struct timlib_priv *t;
    dev = timlib_open(DEV_NAME);
    ASSERT_TRUE(dev != nullptr);
    ASSERT_TRUE(t_->state == 1);
    timlib_close(dev);
    ASSERT_TRUE(t_->state == 0);
    timlib_open(DEV_NAME);
}

TEST_F(cc_timlib_test, start) {
    uint32_t cnt1, cnt2, cnt3;
    uint32_t basefreq;
    int ret = timlib_get_freq(dev_, &basefreq, NULL);
    ASSERT_TRUE(ret == 0);
    std::cout << "Base frequecy: " << basefreq << " HZ\n";
    timlib_set_freq(dev_, 3*basefreq);
    timlib_start(dev_);
    cnt1 = timlib_get_counter(dev_);
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
    cnt2 = timlib_get_counter(dev_);
    ASSERT_TRUE(cnt2 < cnt1);
    std::cout << "cnt1 = " << cnt1 << " cnt2 = " << cnt2 << "\n";
    timlib_restart(dev_);
    cnt3 = timlib_get_counter(dev_);
    ASSERT_TRUE(cnt3 > cnt2);

    timlib_stop(dev_);
    cnt1 = timlib_get_counter(dev_);
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
    cnt2 = timlib_get_counter(dev_);
    ASSERT_TRUE(cnt1 == cnt2);
}

TEST_F(cc_timlib_test, isr) {
    uint32_t basefreq;
    int ret = timlib_get_freq(dev_, &basefreq, NULL);
    ASSERT_TRUE(ret == 0);
    timlib_set_freq(dev_, basefreq);
    ret = timlib_register_intr(dev_, test_timer_isr, (void *)dev_);
    ASSERT_TRUE(ret == 0);
    ASSERT_TRUE(isr_counter == 0);
    timlib_start(dev_);
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(3500));
    ASSERT_TRUE(isr_counter == 3);
}
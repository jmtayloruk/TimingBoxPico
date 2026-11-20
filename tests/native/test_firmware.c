/* Tests include the production translation unit: no copied parser or scheduler.
 * Each test runs in its own process, so globals start in the device's initial
 * state and sanitizer errors/deadlocks cannot hide the rest of the test report.
 */
#define TIMINGBOX_HOST_TEST 1
#include "pico_shim.h"
/* Deterministic poison exposes dependence on uninitialised heap bytes. */
static void *poison_malloc(size_t n) { void *p=malloc(n); if(p) memset(p,0xA5,n); return p; }
#define malloc poison_malloc
#include "../../picoTimingBoxFirmware.c"
#undef malloc
#undef main
#include <math.h>
#include "fixtures.h"
#define CHECK(c,...) do { if(!(c)) { fprintf(stderr,__VA_ARGS__); fputc('\n',stderr); exit(1); } } while(0)
static const uint32_t boardLedPin = 1u<<25;
static const uint32_t ledPins = (1u<<25) | (1u<<26) | (1u<<27);
static void setup(void) {
    queue_init(&commandQ,32,commandQDepth); queue_init(&responseQ,32,responseQDepth); queue_init(&dumpDebugQ,32,32);
    resetPinSources();
    pianolaStatus=READY; context=0;
}
/* Host packets are passed through the actual UART receiver, then the actual
 * manager. Helpers never call CommandParser directly or infer packet lengths. */
static void receive_bytes(const unsigned char *p,unsigned n) {
    CHECK(rx_pos==rx_count,"Test setup: previous UART bytes were not consumed");
    CHECK(n<=sizeof(rx),"Test input too large"); memcpy(rx,p,n);rx_pos=0;rx_count=n;
}
static void step(void) { context=1; pianolaManagerStep(); context=0; }
static void service(void) { context=0; core0Step(); }
static void send_packet(const unsigned char *p,unsigned n) {receive_bytes(p,n);service();step();service();for(unsigned i=0;i<40 && uartTxUsed<uartTxLength;i++) service();}
#define SEND(...) do { const unsigned char packet_bytes[]={__VA_ARGS__};send_packet(packet_bytes,sizeof(packet_bytes)); } while(0)
static void entry(unsigned address,unsigned mask,uint32_t duration) {
    SEND(1,address,mask,duration>>16,duration>>8,duration);
}
static uint32_t expected_mask(unsigned bits) {
    /* Default PIO wiring, before any per-pin override. */
    const unsigned positions[]={0,9,12,13,23,25,24};uint32_t result=0;
    for(unsigned b=0;b<count_of(positions);b++) if(bits&(1u<<b)) result|=1u<<positions[b];return result;
}
static void expect_next(unsigned bits,uint32_t duration,const char *where) {
    for(unsigned i=0;i<8 && fifo_level<2;i++) step();
    CHECK(consume_entry(),"%s: no complete next entry (FIFO has %u words)",where,fifo_level);
    uint32_t countdown = (duration > 1 ? duration : 1) - 1;
    CHECK(active_countdown==countdown && active_mask==expected_mask(bits),
        "%s: expected (countdown=%u, mask=0x%08x), got (%u, 0x%08x)",
        where,countdown,expected_mask(bits),active_countdown,active_mask);
}
static void run(void) { SEND(5); }
static void test_protocol_lengths(void) {
    const unsigned lengths[]={0,6,2,2,2,1,4,1,1,4,2,5,19,3};
    for(unsigned op=1;op<=13;op++) CHECK(BytesToReceive(op)==lengths[op],"Opcode 0x%02x: expected packet length %u, got %d",op,lengths[op],BytesToReceive(op));
    SEND(0xFD);CHECK(tx_count==2 && tx[0]==6 && tx[1]==6,"Firmware version reply must be exactly [6,6]");
}
static void test_masks_durations(void) {
    const uint32_t durations[]={0,1,4,255,256,65535,65536,0x7FFFFF,0xFFFFFF};
    for(unsigned b=0;b<256;b++) {
        uint32_t d=durations[b%count_of(durations)];entry(b,b,d);
        CHECK((uint32_t)pianola[b]==d && (pianola[b]>>32)==b,"Entry %u: wrong mask/duration decoding",b);
    }
}
static void test_camera(void) {
    for(unsigned i=0;i<256;i++) SEND(11,i,0x12,0x34,0x56);
    for(unsigned i=0;i<8;i++) CHECK(pianolaCameraClock[i]==0x123456,"Clock %u: incorrect interval",i);
    SEND(0xFD);CHECK(tx_count==2,"Clock packets lost framing before version query");
}
static void test_piv(void) {
    SEND(12,0,0x12,0x34,0x56,0x78,0x80,0,0,1,0xFF,0xFF,0xFF,0xFF,0,0,0,0,7);
    CHECK(pivIntervalLaserPulses==0x12345678u && pivDuratonPulsePair==0x80000001u && pivIntervalPulseToExp==0xFFFFFFFFu && pivCameraExposure==0 && pivCameraMonitorIdx==7,"PIV parameters: big-endian 32-bit decoding or exposure index incorrect");
    SEND(13,0,1);SEND(13,0,0);SEND(0xFD);CHECK(tx_count==2,"PIV enable/disable must consume both parameters without a reply");
}
static void test_pin_sources(void) {
    const unsigned defaults[]={0,1,2,3,25,29,29};
    for(unsigned p=0;p<256;p++) {
        SEND(9,p,27,1);unsigned before=tx_count;SEND(10,p);
        CHECK(tx_count==before+2 && tx[before]==(p<count_of(defaults)?defaults[p]:29) && tx[before+1]==0,"Pin %u: default readback or unsupported-source handling is inconsistent",p);
    }
    for(unsigned p=0;p<count_of(defaults);p++) {
        for(unsigned source=0;source<8;source++) {
            unsigned inverted=source&1;
            SEND(9,p,source,inverted);unsigned before=tx_count;SEND(10,p);
            CHECK(tx_count==before+2 && tx[before]==source && tx[before+1]==inverted,
                  "Pin %u: did not report configured source %u and inversion",p,source);
        }
    }
}
static void test_time_arithmetic(void) {
    uint32_t seed=0x4D595DF4;
    for(unsigned i=0;i<20000;i++) {
        seed=seed*1664525u+1013904223u;uint32_t now=seed&0xFFFFFF;
        const uint32_t d[]={0,1,3,4,0x7FFFFF,0x800000,0x800001,0xFFFFFF};
        for(unsigned j=0;j<count_of(d);j++) {
            uint32_t target=(now+d[j])&0xFFFFFF;
            CHECK(isTimeInFuture(now,target)==(d[j]>0 && d[j]<0x800000),"24-bit future comparison failed: now=%06x target=%06x",now,target);
        }
    }
}
static void test_reply_time(void) {
    fake_time=0x123456;SEND(8);CHECK(tx_count==3 && tx[0]==0x12 && tx[1]==0x34 && tx[2]==0x56,"Current-time reply must contain the sampled 24-bit time in big-endian order");
}
static void test_scheduled(void) {
    entry(0,1,4);SEND(2,0);fake_time=0xFFFF00;
    SEND(6,0,3,0);CHECK(tx[0]==1,"Scheduled start a few ms ahead across wrap must be accepted");
    fake_time=0x2FF;step();CHECK(submitted_count==0,"Scheduled start executed before its deadline");
    fake_time=0x300;step();expect_next(1,4,"Scheduled start across wrap");
}
static void test_replace_pending(void) {
    entry(0,1,4);fake_time=100;step();SEND(6,0,4,0);SEND(6,0,8,0);
    fake_time=1024;step();CHECK(submitted_count==0,"Replacing a pending start left the old start armed");
    fake_time=2048;step();expect_next(1,4,"Replacement pending start");
}
static void test_reject_cancels(void) {
    fake_time=100;step();SEND(6,0,4,0);SEND(6,0,0,99);
    CHECK(tx_count==8 && tx[0]==1 && tx[4]==0,"Expected acceptance followed by rejection");
    fake_time=1024;step();
    CHECK(!submitted_count,"Rejected past start did not cancel the previously pending start (XMOS contract)");
}
static void test_missed_deadline(void) {
    entry(0,1,4);fake_time=0xFFFF00;SEND(6,0,0,0);
    CHECK(tx[0]==1,"Expected acceptance for a start scheduled at clock wrap");
    fake_time=0xFFFFFC;step();
    CHECK(!submitted_count,"Scheduled start executed before its deadline");
    fake_time=1;step();
    CHECK(submitted_count==2,"First poll after the deadline did not queue the start");
    expect_next(1,4,"Scheduled start on the first poll after its deadline");
    fake_time=2;step();
    CHECK(submitted_count==2,"Late scheduled start fired more than once");
}
static void test_one_shot(void) {
    for(unsigned i=0;i<256;i++) entry(i,i,4+i);
    SEND(2,255);run();
    for(unsigned i=0;i<256;i++) expect_next(i,4+i,"256-entry one-shot");
    for(unsigned i=0;i<8;i++) step();CHECK(fifo_level==0 && submitted_count==512,"One-shot overran final address or duplicated entries");
}
static void test_zero_duration(void) {
    entry(0,1,0);entry(1,3,0);entry(2,2,0xFFFFFF);SEND(2,2);run();
    expect_next(1,0,"Zero-duration first entry");
    expect_next(3,0,"Zero-duration refill");
    expect_next(2,0xFFFFFF,"Maximum-duration refill");
}
static void test_sentinel(void) {
    entry(0,1,4);entry(1,2,781);SEND(2,1);entry(2,31,1);run();
    expect_next(1,4,"Sentinel test entry 0");expect_next(2,781,"Sentinel test entry 1");
    for(unsigned i=0;i<8;i++) step();CHECK(fifo_level==0,"Host's extra entry after SetEnd was executed");
}
static void test_repeat(void) {
    entry(0,1,4);entry(1,2,17);entry(2,4,83);SEND(2,2);SEND(3,1);SEND(4,1);run();
    expect_next(1,4,"Repeat prefix");for(unsigned i=0;i<40;i++) {expect_next(2,17,"Repeat section A");expect_next(4,83,"Repeat section B");}
}
static void test_single_repeat(void) {
    entry(0,1,4);SEND(2,0);SEND(4,1);run();for(unsigned i=0;i<100;i++) expect_next(1,4,"Single-entry loop");
}
static void test_repeat_boundaries(void) {
    const unsigned lengths[]={1,2,17,256};
    for(unsigned nidx=0;nidx<count_of(lengths);nidx++) {
        unsigned n=lengths[nidx];
        for(unsigned variant=0;variant<3;variant++) {
            SEND(7);fifo_level=0;active=false;submitted_count=0;
            for(unsigned i=0;i<n;i++) entry(i,(i*37u)&255u,4+(i%19));
            unsigned from=variant==0?0:variant==1?n/2:n-1;
            SEND(2,n-1);SEND(3,from);SEND(4,1);run();
            for(unsigned i=0;i<n;i++) expect_next((i*37u)&255u,4+i%19,"Initial pass at sequence/repeat boundary");
            for(unsigned pass=0;pass<3;pass++)
                for(unsigned i=from;i<n;i++) expect_next((i*37u)&255u,4+i%19,"Repeat boundary matrix");
            CHECK(!violations,"Repeat matrix overflowed FIFO");
        }
    }
}
static void test_unknown_commands(void) {
    // Undefined single-byte opcodes must not corrupt the following query.
    for(unsigned op=14;op<253;op++) {
        unsigned before=tx_count;SEND(op);SEND(0xFD);
        CHECK(tx_count==before+2 && tx[before]==6 && tx[before+1]==6,"Unknown opcode 0x%02x corrupted subsequent response",op);
    }
}
static void test_scheduling_matrix(void) {
    const unsigned starts[]={0,1,0x7FFFFF,0x800000,0xFFFFFE};
    for(unsigned i=0;i<count_of(starts);i++) {
        SEND(7);fifo_level=0;active=false;submitted_count=0;
        entry(0,1,4);entry(1,2,8);SEND(2,1);fake_time=starts[i];step();
        unsigned target=(fake_time+1000)&0xFFFFFF;
        SEND(6,target>>16,target>>8,target);
        for(unsigned delta=10;delta>0;delta--) {fake_time=(target-delta)&0xFFFFFF;step();CHECK(!submitted_count,"Start occurred too early in scheduling matrix");}
        fake_time=target;step();expect_next(1,4,"Matrix scheduled first entry");expect_next(2,8,"Matrix scheduled second entry");
    }
}
static void test_repeat_off(void) {
    entry(0,1,4);entry(1,2,8);SEND(2,1);SEND(4,1);run();
    expect_next(1,4,"First pass start");
    expect_next(2,8,"First pass end");
    expect_next(1,4,"Repeat pass start");
    CHECK(pianolaStatus==REPEATING,"Did not establish repeating state");
    SEND(4,0);
    expect_next(2,8,"Final entry of the repeat pass");
    step();
    CHECK(pianolaStatus==REPEATING,"Playback ended before the final entry completed");
    active=false;driver_pc=driver_wait_pc;
    step();step();
    CHECK(pianolaStatus==READY && pianolaPosition==0 && fifo_level==0,
          "Disabling repetition did not finish playback after the repeat pass");
}
static uint32_t output_levels(void);
static void finish_and_expect_ready(const char *where, unsigned last_mask) {
    CHECK(fifo_level==0,"%s: invalid repeat queued a successor",where);
    step();
    CHECK(pianolaStatus==RUNNING || pianolaStatus==REPEATING,"%s: ready before active final entry completed",where);
    active=false;driver_pc=driver_wait_pc;step();
    CHECK(pianolaStatus==READY && pianolaPosition==0 && fifo_level==0,"%s: did not finish at READY",where);
    CHECK(output_levels()==(expected_mask(last_mask&0xF)<<2),"%s: changed final pattern instead of natural completion",where);
    for(unsigned i=0;i<10;i++) step();
    CHECK(!fifo_level && pianolaStatus==READY,"%s: stopped sequence spontaneously repeated",where);
}
static void test_invalid_repeat(void) {
    entry(0,1,4);SEND(2,0);SEND(3,255);SEND(4,1);run();expect_next(1,4,"Invalid-repeat first entry");
    finish_and_expect_ready("Invalid repeat beyond final",1);
    CHECK(pianolaRepeatFlag==1 && pianolaRepeatAddr==255,"Boundary validation should not rewrite uploaded settings");
    SEND(0xFD);CHECK(tx_count>=2 && tx[tx_count-2]==6 && tx[tx_count-1]==6,"Invalid repeat prevented further command service");
}
static void test_invalid_repeat_boundaries(void) {
    const unsigned finals[]={0,1,127,254};
    for(unsigned f=0;f<count_of(finals);f++) for(unsigned variant=0;variant<2;variant++) {
        SEND(7);unsigned final=finals[f];
        for(unsigned i=0;i<=final;i++) entry(i,(i+1)&31,4);
        SEND(2,final);SEND(3,variant?255:final+1);SEND(4,1);run();
        for(unsigned i=0;i<=final;i++) expect_next((i+1)&31,4,"Invalid-repeat boundary matrix");
        finish_and_expect_ready("Invalid-repeat boundary matrix",(final+1)&31);
    }
}
static void test_repeat_before_final_upload(void) {
    // A temporarily invalid configuration while stopped is a valid upload order.
    entry(0,1,4);entry(1,2,8);entry(2,4,16);
    SEND(3,2);SEND(4,1);SEND(2,2);run();
    expect_next(1,4,"Upload-order prefix zero");expect_next(2,8,"Upload-order prefix one");
    for(unsigned i=0;i<4;i++) expect_next(4,16,"RepeatFrom preserved until final supplied");
}
static void test_live_repeat_range_invalidation(void) {
    entry(0,1,4);entry(1,2,8);entry(2,4,16);SEND(2,2);SEND(3,1);SEND(4,1);run();
    expect_next(1,4,"Live repeat edit prefix");SEND(3,255);
    expect_next(2,8,"Live repeat edit remaining entry one");expect_next(4,16,"Live repeat edit final entry");
    finish_and_expect_ready("Repeat address invalidated while playing",4);
}
static void test_shrink_final_during_repeat(void) {
    entry(0,1,4);entry(1,2,8);entry(2,4,16);SEND(2,2);SEND(3,2);SEND(4,1);run();
    expect_next(1,4,"Shrink prefix zero");expect_next(2,8,"Shrink prefix one");expect_next(4,16,"Shrink prefix end");
    expect_next(4,16,"Establish active repeat section");CHECK(pianolaStatus==REPEATING,"Expected repeating state before shrinking final");
    SEND(2,1);
    // The old entry is already active: let it finish; do not jump to entry zero.
    finish_and_expect_ready("Final shortened below repeat address",4);
    run();expect_next(1,4,"Restart shortened sequence zero");expect_next(2,8,"Restart shortened sequence final");
    finish_and_expect_ready("Restart with still-invalid repeat configuration",2);
}
static void test_repair_repeat_range_before_boundary(void) {
    entry(0,1,4);entry(1,2,8);SEND(2,1);SEND(3,255);SEND(4,1);run();
    expect_next(1,4,"Repair prefix");expect_next(2,8,"Repair final entry");step();
    SEND(3,1);
    for(unsigned i=0;i<4;i++) expect_next(2,8,"Repaired configuration repeats normally");
}
static void test_stop_pending(void) {
    fake_time=100;step();SEND(6,0,4,0);SEND(7);fake_time=1024;step();CHECK(fifo_level==0 && !pianolaFirePending,"Stop failed to cancel scheduled start");
}
static void prepare_active(void) {
    entry(0,1,1000);entry(1,2,4);SEND(2,1);run();expect_next(1,1000,"Active-entry setup");step();CHECK(fifo_level==2,"Test setup expected queued successor");
}
static void test_stop_queued(void) {
    prepare_active();SEND(7);
    CHECK(fifo_level==0,"Stop left a successor in PIO FIFO; it can change outputs after stop");
}
static void test_restart(void) {
    prepare_active();run();
    CHECK(!(active && active_countdown==999 && fifo_level>=2 && fifo[1]==expected_mask(2)),"Restart left old active/queued work ahead of new entry zero");
    expect_next(1,1000,"Restart must begin at entry zero");
}
static void test_uart_owner(void) {
    SEND(0xFD);CHECK(!uart_context_errors,"Core 1 accessed UART %u times; UART must have one core owner",uart_context_errors);
}
static void test_queue_burst(void) {
    const unsigned char burst[]={1,0,1,0,0,4,1,1,2,0,0,8,2,1};receive_bytes(burst,sizeof(burst));
    /* Simulate core 0 receiving three complete no-response commands before core 1
     * gets scheduled. Serialisation does not imply an ACK between these packets. */
    for(unsigned i=0;i<3;i++) service();
    for(unsigned i=0;i<6;i++) step();
    CHECK((uint32_t)pianola[0]==4 && (uint32_t)pianola[1]==8 && pianolaFinalPosition==1,"Back-to-back upload lost commands (%u rejected queue writes)",dropped);
}
static void test_partial_packet(void) {
    const unsigned char prefix[]={1,0};receive_bytes(prefix,sizeof(prefix));trap_enabled=true;
    if(setjmp(trap)==0) service();else {trap_enabled=false;CHECK(false,"Incomplete packet blocks UART service: %s",trap_reason);}
    trap_enabled=false;
    const unsigned char rest[]={1,0,0,4};receive_bytes(rest,sizeof(rest));service();step();
    CHECK((uint32_t)pianola[0]==4,"Fragmented sequence packet was not reassembled");
}
static void test_debug_dump(void) {
    /* Poisoned allocation is not assumed; empty capture must be all zero by
     * contract. This comparison detects incorrect padding even without MSan. */
    SEND(0xFE);CHECK(tx_count==1024,"Debug dump has incorrect length: %u",tx_count);
    for(unsigned i=0;i<1024;i++) CHECK(tx[i]==0,"Empty debug dump contains nonzero/uninitialised byte at offset %u",i);
}
static void test_debug_record(void) {
    SEND(0xFD);unsigned char record[32];CHECK(queue_try_remove(&dumpDebugQ,record),"Version response missing from log");
    CHECK(record[0]==2 && record[1]==6 && record[2]==6,"Debug record omitted response data: expected length,6,6; got %u,%u,%u",record[0],record[1],record[2]);
}
static void test_configuration(void) {
    uint32_t mask=(1u<<2)|(1u<<11)|(1u<<14)|(1u<<15)|ledPins;
    // Execute the real startup, including its actual configuration arguments.
    // Stop before either infinite core loop starts; no startup copy lives here.
    trap_enabled=true;
    if(setjmp(trap)==0) firmware_main();
    trap_enabled=false;
    CHECK(!strcmp(trap_reason,"firmware startup reached core-1 launch"),"Startup failed before launching core 1: %s",trap_reason);
    CHECK(output_base==2 && output_count==26,"PIO OUT mapping does not span all seven outputs");
    CHECK(gpio_directions==mask && gpio_functions==mask && !(gpio_levels&mask),"Initialisation drove/muxed unintended GPIOs or failed to initialise low");
    CHECK(gpio_overridden==ledPins && !(gpio_forced_high&ledPins),"Startup did not drive all three configurable LEDs low");
}
static void test_live_mutation(void) {
    uint32_t seed=0x12345678;
    for(unsigned i=0;i<256;i++) entry(i,i,4+i);
    SEND(2,255);SEND(4,1);run();
    for(unsigned i=0;i<2000;i++) {
        seed=seed*1664525u+1013904223u;
        switch(seed%8) {
        case 0: entry(seed>>24,seed>>16,seed&0xFFFFFF);break;
        case 1: SEND(2,seed>>24);break;
        case 2: SEND(3,seed>>24);break;
        case 3: SEND(4,seed&1);break;
        case 4: SEND(8);break;
        case 5: SEND(7);run();break;
        default: step();break;
        }
        if(fifo_level>=2) consume_entry();
        CHECK(pianolaPosition>=0 && pianolaPosition<=256 && !violations,"Seed 0x12345678 event %u: invalid index/FIFO overflow",i);
        if(tx_count>8000) tx_count=0;
    }
    SEND(7);SEND(0xFD);CHECK(tx_count>=2,"Live changes prevented subsequent stop/query service");
}
static void test_workloads(void) {
    for(unsigned f=0;f<count_of(fixtures);f++) {
        const fixture *x=&fixtures[f];
        /* This is the host's real ordering, including the extra unwitnessed
         * sentinel after SetEnd. Repeat workloads are checked over three passes. */
        SEND(7);fifo_level=0;active=false;submitted_count=0;
        for(unsigned i=0;i<x->count;i++) entry(i,x->entries[i].mask,x->entries[i].duration);
        SEND(4,x->endless);SEND(2,x->count-1);entry(x->count,0,1);
        fake_time=100;step();SEND(6,0,4,0);fake_time=1024;step();
        for(unsigned pass=0;pass<(x->endless?3u:1u);pass++)
            for(unsigned i=0;i<x->count;i++) expect_next(x->entries[i].mask,x->entries[i].duration,x->name);
        if(!x->endless) {for(unsigned i=0;i<8;i++) step();CHECK(!fifo_level,"%s: executed extra sentinel",x->name);}
    }
}
static void test_calibration(void) {
    /* Matches CameraTiming.mm: low 1, trigger 100, low (delay-100),
     * trigger 100, low 1; SetEnd; then an excluded sentinel. */
    unsigned masks[]={0,1,0,1,0};uint32_t d[]={1,100,390625-100,100,1};
    for(unsigned i=0;i<5;i++) entry(i,masks[i],d[i]);SEND(2,4);entry(5,31,1);run();
    for(unsigned i=0;i<5;i++) expect_next(masks[i],d[i],"Timebase calibration");
    for(unsigned i=0;i<8;i++) step();CHECK(!fifo_level,"Calibration ran past final entry");
}
static void test_self_trigger(void) {
    /* Representative host parameters: 10 Hz, 10 ms readout, 1% trigger high. */
    unsigned masks[]={0,3,0};uint32_t d[]={3906,391,34766};
    for(unsigned i=0;i<3;i++) entry(i,masks[i],d[i]);SEND(4,1);SEND(2,2);entry(3,31,1);run();
    for(unsigned i=0;i<30;i++) expect_next(masks[i%3],d[i%3],"Self-trigger cycle");
}
/* Explicit servicing-delay experiment, not a CPU-speed estimate. The fake
 * consumer uses requested durations (corrected PIO semantics). A long entry
 * precedes ONE short entry; service happens every four ticks, at phase +1.
 * This exposes the FIFO's lookahead limit without requiring short entries in a row.
 */
static void refill_experiment(unsigned short_ticks) {
    entry(0,1,100);entry(1,2,short_ticks);entry(2,4,100);SEND(2,2);run();
    expect_next(1,100,"Refill experiment prefix");step();
    unsigned entry_index=0, deadline=100, late=0;
    for(unsigned t=1;t<=220;t++) {
        fake_time=t;
        // Consumption before service is a valid adversarial ordering at equal time.
        if(t>=deadline && entry_index<2) {
            if(consume_entry()) {
                if(entry_index==1) late=t-deadline;
                entry_index++;deadline=t+(entry_index==1?short_ticks:100);
            }
        }
        if(t%4==1) step();
    }
    CHECK(entry_index==2,"Refill experiment never received successor");
    CHECK(!late,"Single %u-tick entry delayed its successor by %u ticks under an ASSUMED four-tick servicing interval (not measured CPU speed)",short_ticks,late);
}
static void test_refill_four(void) { refill_experiment(4); }
static void test_refill_one(void) { refill_experiment(1); }

// Ownership is not enough: execute the actual startup through its first core-0
// foreground iteration, verifying polling exists and no UART IRQ can reenter it.
static void test_core0_foreground(void) {
    startup_through_core0=true;trap_enabled=true;
    if(setjmp(trap)==0) firmware_main();
    trap_enabled=false;
    CHECK(!strcmp(trap_reason,"core-0 foreground loop reached"),"Did not reach core-0 polling loop");
    CHECK(!irq_handler_registrations && !uart_irq_enabled && !cpu_uart_irq_enabled,"UART polling still has a competing interrupt handler");
}
static void test_idle_response_poll(void) {
    const unsigned char query[]={0xFD};receive_bytes(query,1);service();
    CHECK(!tx_count,"Reply existed before command was processed");step();
    for(unsigned i=0;i<10 && tx_count<2;i++) service();
    CHECK(tx_count==2 && tx[0]==6 && tx[1]==6,"Isolated query requires another received byte to get its reply");
    CHECK(!uart_context_errors,"Reply processing accessed UART from core 1");
}
static void test_tx_backpressure(void) {
    tx_capacity=0;SEND(0xFD);
    CHECK(!tx_count,"Blocked TX must not send bytes");
    const unsigned char partial[]={1,7};receive_bytes(partial,2);service();
    CHECK(rx_pos==2,"Blocked TX prevented RX progress");
    tx_capacity=sizeof(tx);service();
    CHECK(tx_count==2 && tx[0]==6 && tx[1]==6,"Paused TX reply was lost/corrupted");
    const unsigned char rest[]={1,0,0,4};receive_bytes(rest,4);service();step();
    CHECK((uint32_t)pianola[7]==4,"Partial command damaged while TX was blocked");
}
static void test_rx_backpressure(void) {
    unsigned char filler[32]={4,0};for(unsigned i=0;i<commandQDepth;i++) CHECK(queue_try_add(&commandQ,filler),"Cannot fill command queue");
    const unsigned char packet[]={1,7,1,0,0,4};receive_bytes(packet,6);service();
    CHECK(uartRxUsed==6 && (uint32_t)pianola[7]==0,"Full queue did not retain completed command");
    step();service();
    for(unsigned i=0;i<commandQDepth+2;i++) step();
    CHECK((uint32_t)pianola[7]==4,"Retained command lost after queue space became available");
}
static void test_response_backpressure(void) {
    // Responses saturate while TX is unavailable. No command gets a duplicate
    // or missing reply after the consumer resumes. Playback can still step.
    tx_capacity=0;
    for(unsigned i=0;i<responseQDepth+3;i++) SEND(0xFD);
    tx_capacity=sizeof(tx);
    for(unsigned i=0;i<200;i++) {service();step();}
    CHECK(tx_count==2*(responseQDepth+3),"Response backpressure lost/duplicated data: expected %u bytes, got %u",2*(responseQDepth+3),tx_count);
    for(unsigned i=0;i<tx_count;i++) CHECK(tx[i]==6,"Backpressured response byte %u is wrong",i);
}
static void test_serial_interleavings(void) {
    const unsigned char packets[]={1,0,1,0,0,4,1,1,2,0,0,8,2,1,0xFD,8};
    // Byte-at-a-time delivery with producer/consumer bursts in a reproducible
    // order catches framing and progress bugs beyond specific known packets.
    uint32_t seed=0x87654321;
    for(unsigned i=0;i<sizeof(packets);i++) {
        receive_bytes(packets+i,1);service();seed=seed*1664525u+1013904223u;
        for(unsigned j=0;j<(seed%4);j++) step();
    }
    for(unsigned i=0;i<40;i++) {step();service();}
    CHECK((uint32_t)pianola[0]==4 && (uint32_t)pianola[1]==8 && pianolaFinalPosition==1,"Alternating core schedules corrupted programming commands");
    CHECK(tx_count==5 && tx[0]==6 && tx[1]==6,"Alternating core schedules lost or reordered replies");
    CHECK(!block_on_empty && !uart_context_errors,"Fragmented/interleaved delivery caused blocking or wrong-core access");
}

static void test_full_wire_upload(void) {
    unsigned char wire[256*6+3];unsigned n=0;
    for(unsigned i=0;i<256;i++) {
        uint32_t d=4+i;unsigned char p[]={1,i,i,d>>16,d>>8,d};
        memcpy(wire+n,p,sizeof(p));n+=sizeof(p);
    }
    wire[n++]=2;wire[n++]=255;wire[n++]=0xFD;
    receive_bytes(wire,n);
    // Producer gets bursts of foreground iterations; consumer gets only one.
    // This fills the queues and exercises retention without a copied parser.
    for(unsigned i=0;i<1000;i++) {for(unsigned j=0;j<3;j++) service();step();}
    for(unsigned i=0;i<256;i++) CHECK((uint32_t)pianola[i]==4+i && (pianola[i]>>32)==i,"Full wire upload corrupted entry %u",i);
    CHECK(rx_pos==rx_count && pianolaFinalPosition==255 && tx_count==2 && tx[0]==6 && tx[1]==6,"Full upload lost end marker or following reply");
}
static void test_debug_wire_order(void) {
    const unsigned char commands[]={0xFE,0xFD};receive_bytes(commands,2);service();step();step();
    for(unsigned i=0;i<80;i++) service();
    CHECK(tx_count==1026 && tx[1024]==6 && tx[1025]==6,"Debug dump and subsequent reply interleaved or were lost");
    CHECK(!uart_context_errors,"Debug request caused UART access from core 1");
}

static uint32_t observed_gpio_levels(void) {
    return (gpio_levels&~gpio_overridden)|(gpio_forced_high&gpio_overridden);
}

// Compare BNC levels independently of the configurable LED routes.
static uint32_t output_levels(void) {
    return observed_gpio_levels() & ((1u<<2)|(1u<<11)|(1u<<14)|(1u<<15));
}
static void test_live_pin_routing(void) {
    entry(0,1,20);entry(1,1,30);entry(2,0x82,40);SEND(2,2);run();
    expect_next(1,20,"Active entry before reroute");
    step();
    SEND(9,0,1,0);
    CHECK(output_levels()==(1u<<2) && fifo_level==2,"Remap interrupted the active entry or flushed the FIFO");
    expect_next(1,30,"Queued entry retains original mapping");
    expect_next(3,40,"Later entry routes bit 1 to both BNCs");

    SEND(9,0,7,1);
    CHECK(output_levels()==((1u<<2)|(1u<<11)),"Remap changed the final entry before it completed");
    entry(2,0,1);
    active=false;
    driver_pc=driver_wait_pc;
    step();
    CHECK(pianolaStatus==READY && output_levels()==(1u<<11),
          "Completion did not reroute the held snapshot with bit 7 inverted");
    SEND(9,0,7,0);
    CHECK(output_levels()==((1u<<2)|(1u<<11)),"Idle remap did not immediately use the held snapshot");
}
static void test_one_hz_routing(void) {
    SEND(9,0,7,0);
    entry(0,0x80,20);entry(1,0x81,30);SEND(2,1);run();
    expect_next(1,20,"Bit 7 drives BNC before clock selection");

    SEND(9,0,12,0);
    unsigned before=tx_count;SEND(10,0);
    CHECK(tx_count==before+2 && tx[before]==12 && tx[before+1]==0 && output_levels()==0,
          "Clock selection did not report source 12 and immediately apply the low phase");
    SEND(9,1,12,1);
    CHECK(output_levels()==(1u<<11),"Inverted output did not share the clock's phase");
    fake_time_us=499999;step();
    CHECK(output_levels()==(1u<<11),"First clock edge arrived early");
    fake_time_us=500000;step();
    CHECK(output_levels()==(1u<<2),"Clock did not toggle both outputs at a half-second");
    expect_next(1,30,"PIO still drives the routed bit behind the override");
    fake_time_us=1000000;step();
    CHECK(output_levels()==(1u<<11),"Clock missed its falling edge while playing");
    SEND(9,0,7,0);
    CHECK(gpio_overridden==(ledPins|(1u<<11)) && output_levels()==((1u<<2)|(1u<<11)),
          "Returning to an unchanged pianola route did not immediately unmask the high level");

    SEND(7);
    fake_time_us=1500000;step();
    CHECK(output_levels()==(1u<<2),"Clock stopped with the pianola");
    fake_time_us=3250000;step();
    CHECK(output_levels()==((1u<<2)|(1u<<11)),"Clock did not recover its absolute phase after a late poll");
    SEND(0xFF);
    before=tx_count;SEND(10,0);SEND(10,1);
    CHECK(gpio_overridden==ledPins && !observed_gpio_levels() && tx_count==before+4 && tx[before]==0 && tx[before+2]==1,
          "Hard reset did not clear overrides and restore default routes");
}
static void test_constant_pin_sources(void) {
    const uint32_t bnc0 = 1u << 2;
    const uint32_t bnc1 = 1u << 11;
    entry(0,1,20);entry(1,2,30);SEND(2,1);run();
    expect_next(1,20,"Active entry before constant override");

    SEND(9,0,12,0);
    CHECK(output_levels()==0,"Initial clock state did not override BNC 0");
    SEND(9,0,28,0);SEND(9,1,29,0);
    expect_next(2,30,"Pianola advances behind both constant overrides");
    fake_time_us=1000000;step();
    CHECK(output_levels()==bnc0,"The clock or the queued pianola mask changed a constant output");

    SEND(9,0,28,1);SEND(9,1,29,1);
    unsigned before=tx_count;SEND(10,0);SEND(10,1);
    CHECK(output_levels()==bnc1 && tx_count==before+4 && tx[before]==28 && tx[before+1]==1 &&
          tx[before+2]==29 && tx[before+3]==1,"Constant source readback or inversion is incorrect");

    SEND(9,0,29,1);SEND(9,1,28,1);
    CHECK(output_levels()==bnc0,"Inverted constants did not drive the opposite levels");
    SEND(9,0,0,0);SEND(9,1,1,0);
    CHECK(gpio_overridden==ledPins && output_levels()==bnc1,"Clearing constant overrides did not reveal the active pianola pattern");

    SEND(9,0,28,0);SEND(9,1,29,0);SEND(0xFF);
    CHECK(gpio_overridden==ledPins && !observed_gpio_levels(),"Hard reset left a custom constant override active");
}
static void test_pianola_status_sources(void) {
    const uint32_t running = 1u << 2;
    const uint32_t pending = 1u << 11;
    const uint32_t invertedPending = 1u << 14;
    entry(0,0,40);SEND(2,0);
    SEND(9,0,25,0);SEND(9,1,26,0);SEND(9,2,26,1);
    unsigned before=tx_count;SEND(10,0);SEND(10,1);SEND(10,2);
    CHECK(tx_count==before+6 && tx[before]==25 && tx[before+1]==0 &&
          tx[before+2]==26 && tx[before+3]==0 && tx[before+4]==26 && tx[before+5]==1,
          "Status routing did not round-trip the source and inversion");
    CHECK(output_levels()==invertedPending,"Idle Running and Pending levels are incorrect");

    fake_time=100;SEND(6,0,0,200);
    CHECK(output_levels()==pending,"Arming a start did not assert Pending only");
    SEND(6,0,0,99);
    CHECK(output_levels()==invertedPending,"Rejecting a fire time did not clear Pending");
    SEND(6,0,0,200);
    fake_time=200;step();
    CHECK(output_levels()==(running|invertedPending),"Scheduled start did not transition from Pending to Running");
    expect_next(0,40,"Scheduled status test entry");
    step();
    CHECK(output_levels()==(running|invertedPending),"Running cleared while the final entry was still active");
    active=false;driver_pc=driver_wait_pc;step();
    CHECK(output_levels()==invertedPending,"Running did not clear after the final entry completed");

    SEND(4,1);run();
    CHECK(output_levels()==(running|invertedPending),"Immediate start did not assert Running");
    expect_next(0,40,"First pass before repeat");
    step();
    CHECK(pianolaStatus==REPEATING && output_levels()==(running|invertedPending),"Running did not stay high through repeat");
    SEND(6,0,1,44);
    CHECK(output_levels()==(running|pending),"Running and Pending did not coexist for a scheduled restart");
    SEND(9,3,26,0);
    CHECK(output_levels()==(running|pending|(1u<<15)),"Selecting Pending while armed did not drive its current level");

    fake_time=300;step();
    CHECK(output_levels()==(running|invertedPending),"Scheduled restart did not clear Pending while keeping Running high");
    SEND(6,0,1,144);SEND(7);
    CHECK(output_levels()==invertedPending,"Stopping with a restart armed did not clear both status signals");
    SEND(0xFF);
    CHECK(gpio_overridden==ledPins && !observed_gpio_levels(),"Hard reset did not restore default LED overrides");
}
static void test_led_routing(void) {
    const uint32_t green=1u<<26;
    const uint32_t blue=1u<<27;
    entry(0,0,40);SEND(2,0);SEND(4,1);
    fake_time=100;SEND(6,0,0,200);
    CHECK(!observed_gpio_levels(),"Default LEDs lit while playback was pending");
    fake_time=200;step();
    CHECK(observed_gpio_levels()==boardLedPin,"Only the Board LED should light during playback");
    expect_next(0,40,"First pass with default LED routes");
    step();
    CHECK(pianolaStatus==REPEATING && observed_gpio_levels()==boardLedPin,"Board LED did not stay on through repeat");
    SEND(6,0,1,44);
    CHECK(observed_gpio_levels()==boardLedPin,"An external LED lit when a restart was armed");
    SEND(7);
    CHECK(!observed_gpio_levels(),"Stopping did not turn off the Board LED");

    SEND(9,5,0,0);SEND(9,6,1,0);
    entry(0,1,40);entry(1,2,40);SEND(2,1);run();
    CHECK(gpio_overridden==boardLedPin,"Selecting pianola sources did not release the external LEDs to PIO");
    CHECK(consume_entry() && observed_gpio_levels()==(boardLedPin|blue|(1u<<2)),"Pianola bit 0 did not drive the blue LED and BNC together");
    step();
    CHECK(consume_entry() && observed_gpio_levels()==(boardLedPin|green|(1u<<11)),"Pianola bit 1 did not drive the green LED and BNC together");

    SEND(6,0,1,44);SEND(0xFF);
    unsigned before=tx_count;SEND(10,4);SEND(10,5);SEND(10,6);
    CHECK(gpio_overridden==ledPins && !observed_gpio_levels() && tx_count==before+6 &&
          tx[before]==25 && tx[before+1]==0 && tx[before+2]==29 && tx[before+3]==0 &&
          tx[before+4]==29 && tx[before+5]==0,"Hard reset did not restore all three LED defaults at idle");
}
static void test_hardreset(void) {
    entry(0,31,1000);
    entry(1,2,4);
    entry(255,4,42);
    SEND(2,255);
    SEND(3,1);
    SEND(4,1);
    SEND(11,7,0x12,0x34,0x56);
    SEND(12,3,0,0,0,1,0,0,0,2,0,0,0,3,0,0,0,4,7);
    fake_time=100;
    run();
    expect_next(31,1000,"Entry active before hard reset");
    SEND(6,0,4,0);
    CHECK(active && fifo_level==2 && observed_gpio_levels()==(expected_mask(31)<<2) &&
          pianolaFirePending && pianolaFireTime==1024,"Hard-reset test did not establish active, queued and scheduled work");

    unsigned before=tx_count;
    SEND(0xFF);
    CHECK(tx_count==before,"Hard reset sent an unexpected reply");
    CHECK(output_levels()==0,"Hard reset left outputs high");
    CHECK(!active && !sm_enabled[0] && !fifo_level,"Hard reset retained active or queued output work");
    CHECK(pianolaStatus==INIT && pianolaPosition==0 && pianolaFinalPosition==0 && pianolaNumberOfEntries==1,
          "Hard reset did not restore initial playback state");
    CHECK(!pianolaRepeatAddr && !pianolaRepeatFlag && !pianolaFirePending && !pianolaFireTime,
          "Hard reset retained repeat or scheduled-start configuration");
    for(unsigned i=0;i<count_of(pianola);i++)
        CHECK(!pianola[i],"Hard reset retained entry %u",i);
    for(unsigned i=0;i<count_of(pianolaCameraClock);i++)
        CHECK(!pianolaCameraClock[i],"Hard reset retained camera clock %u",i);
    CHECK(!pivIdx && !pivIntervalLaserPulses && !pivDuratonPulsePair &&
          !pivIntervalPulseToExp && !pivCameraExposure && !pivCameraMonitorIdx,
          "Hard reset retained PIV parameters");

    unsigned submissions_after_reset=submitted_count;
    fake_time=1025;
    step();
    CHECK(submitted_count==submissions_after_reset && !sm_enabled[0] && !output_levels(),
          "Old scheduled start fired after hard reset");

    const unsigned char startup[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x08};
    receive_bytes(startup,sizeof(startup));
    service();
    for(unsigned i=0;i<sizeof(startup);i++) step();
    service();
    CHECK(tx_count==before+3 && tx[before]==0 && tx[before+1]==4 && tx[before+2]==1,
          "Clock query after the reset burst did not return the current clock");
    CHECK(!clock_control_changes && sm_enabled[1],"Hard reset modified the independent clock state machine");

    run();
    expect_next(0,0,"Run after hard reset uses the cleared entry zero");
    active=false;
    driver_pc=driver_wait_pc;
    step();
    CHECK(pianolaStatus==READY && !fifo_level && !output_levels(),
          "Run after hard reset replayed or repeated old entries");
}
static void test_stop_freezes_entry_zero(void) {
    SEND(9,4,4,0);
    entry(0,17,0xFFFFFF);entry(1,2,1000);entry(2,4,1000);SEND(2,2);run();
    expect_next(17,0xFFFFFF,"Entry-zero start");step();expect_next(2,1000,"Different active pattern");step();
    CHECK(fifo_level==2,"Expected queued successor before stop");
    SEND(7);
    uint32_t expected=expected_mask(17)<<2;
    CHECK(observed_gpio_levels()==expected,"Stop must apply nonzero entry-zero pattern, expected GPIO=0x%x got 0x%x",expected,gpio_levels);
    CHECK(!active && !sm_enabled[0] && !fifo_level,"Stop retained active/queued work or left output SM enabled");
    CHECK(pianolaPosition==0 && pianolaStatus==READY && !pianolaFirePending && !pianolaRepeatFlag,"Stop did not rewind/cancel pending/repeat state");
    for(unsigned i=0;i<100;i++) {fake_time+=100;step();CHECK(!consume_entry(),"Stopped driver consumed a new entry");}
    CHECK(observed_gpio_levels()==expected,"Stopped entry-zero pattern was not held indefinitely");
    CHECK(!clock_control_changes && sm_enabled[1],"Stop modified the independent clock state machine");
}
static void test_stop_inflight_matrix(void) {
    SEND(9,4,4,0);
    const unsigned masks[]={0,1,16,31};offsetPinDriver=3;
    // PCs cover status polling, each PULL/OUT boundary, countdown and pin-clear.
    // Odd FIFO levels represent a duration already pulled with mask still queued.
    for(unsigned m=0;m<count_of(masks);m++) for(unsigned pc=0;pc<10;pc++) for(unsigned words=0;words<=4;words++) {
        entry(0,masks[m],0xFFFFFF);sm_enabled[0]=true;active=true;
        driver_pc=offsetPinDriver+pc;driver_delay=31;driver_osr=0xDEADBEEF;driver_shift_count=19;
        gpio_levels=expected_mask(2)<<2;fifo_level=words;
        for(unsigned i=0;i<words;i++) fifo[i]=0xBAD00000+i;
        unsigned controls_before=control_count;SEND(7);
        CHECK(!sm_enabled[0] && !active && !fifo_level && !driver_delay,"Stop left in-flight state at PC=%u FIFO=%u",pc,words);
        CHECK(driver_pc==offsetPinDriver+pinDriver_wrap_target,"Stop failed to reposition PC from %u",pc);
        CHECK(observed_gpio_levels()==(expected_mask(masks[m])<<2),"Stop wrong entry-zero pattern for mask %u, PC %u, FIFO %u",masks[m],pc,words);
        CHECK(controls[controls_before].op=='E' && !controls[controls_before].value && controls[controls_before+1].op=='C' && controls[controls_before+2].op=='R',"Cancellation must disable, clear, restart in order");
    }
    CHECK(!clock_control_changes,"Cancellation matrix disturbed clock SM");
}
static void test_restart_inflight_matrix(void) {
    offsetPinDriver=3;entry(0,1,1000);entry(1,2,4);SEND(2,1);
    for(unsigned pc=0;pc<10;pc++) for(unsigned words=0;words<=4;words++) {
        sm_enabled[0]=true;active=true;driver_pc=offsetPinDriver+pc;
        driver_delay=31;driver_osr=0xBAD;driver_shift_count=19;
        gpio_levels=expected_mask(1)<<2;fifo_level=words;
        for(unsigned i=0;i<words;i++) fifo[i]=0xBAD00000+i;
        unsigned before=control_count;run();
        CHECK(!active && !driver_delay && !driver_shift_count,"Restart resumed old in-flight state at PC=%u FIFO=%u",pc,words);
        CHECK(driver_pc==offsetPinDriver+pinDriver_wrap_target && sm_enabled[0],"Restart did not enable at the wait-loop entry");
        CHECK(fifo_level==2 && fifo[0]==999 && fifo[1]==expected_mask(1),"Restart retained old FIFO words");
        CHECK(output_levels()==(expected_mask(1)<<2),"Restart inserted an output change before entry-zero OUT");
        CHECK(controls[before].op=='E' && !controls[before].value && controls[before+1].op=='C' && controls[before+2].op=='R',"Restart cancellation ordering incorrect");
        CHECK(controls[before+4].op=='P' && controls[before+5].op=='P' && controls[before+6].op=='E' && controls[before+6].value,"Restart must preload a whole pair before enabling");
        expect_next(1,1000,"Restart matrix entry zero");step();expect_next(2,4,"Restart matrix entry one");
    }
    CHECK(!clock_control_changes,"Restart matrix disturbed clock SM");
}
static void test_restart_repeating(void) {
    entry(0,1,4);entry(1,2,8);SEND(2,1);SEND(4,1);run();
    for(unsigned i=0;i<6;i++) expect_next(i%2?2:1,i%2?8:4,"Establish repetition");
    step();CHECK(pianolaStatus==REPEATING,"Did not establish repeating state");run();
    for(unsigned i=0;i<10;i++) expect_next(i%2?2:1,i%2?8:4,"Restart retains configured repetition");
}
static void test_scheduled_restart_discards_old(void) {
    prepare_active();fake_time=100;step();SEND(6,0,4,0);
    CHECK(active && fifo_level==2,"Scheduling cancelled existing in-flight work before deadline");
    fake_time=1024;step();
    CHECK(!active && fifo_level==2 && fifo[0]==999 && fifo[1]==expected_mask(1),"Scheduled firing retained old active/queued work");
    expect_next(1,1000,"Scheduled restart entry zero");
}
static void test_rejected_restart_preserves_playback(void) {
    entry(0,1,4);entry(1,2,8);entry(2,4,16);SEND(2,2);run();
    expect_next(1,4,"Active sequence first entry");
    fake_time=100;SEND(6,0,4,0);
    expect_next(2,8,"Active sequence continues while restart is armed");
    SEND(6,0,0,99);
    fake_time=1024;step();
    expect_next(4,16,"Active sequence continues after pending restart is rejected");
}
static void test_stop_then_start(void) {
    SEND(9,4,4,0);
    entry(0,16,4);entry(1,1,8);SEND(2,1);
    for(unsigned cycle=0;cycle<50;cycle++) {
        run();expect_next(16,4,"Repeated start");step();SEND(7);
        CHECK(!sm_enabled[0] && observed_gpio_levels()==boardLedPin,"Repeated stop failed to freeze LED-high entry zero");
    }
}
static void test_completion_not_fifo_empty(void) {
    entry(0,1,1000);SEND(2,0);run();expect_next(1,1000,"Final entry in flight");step();
    CHECK(pianolaStatus==RUNNING && (observed_gpio_levels()&boardLedPin),"FIFO consumption was incorrectly treated as final-entry completion");
    active=false;driver_pc=driver_wait_pc;step();
    CHECK(pianolaStatus==READY && !(observed_gpio_levels()&boardLedPin),"Finished final entry did not transition to READY at the wait loop");
}
static void test_stop_after_ready(void) {
    SEND(9,4,4,0);
    entry(0,16,4);entry(1,1,8);SEND(2,1);run();
    expect_next(16,4,"Final-state setup 0");expect_next(1,8,"Final-state setup 1");
    active=false;driver_pc=driver_wait_pc;step();CHECK(pianolaStatus==READY,"Expected ready state");
    SEND(7);CHECK(observed_gpio_levels()==boardLedPin && !sm_enabled[0],"Stop while already ready did not apply entry-zero pattern");
}

typedef struct {const char *name,*area;void (*fn)(void);} test;
static const test tests[]={
 {"stop_freeze_entry_zero","Control / apply and indefinitely hold entry zero, including nonzero outputs",test_stop_freezes_entry_zero},
 {"stop_inflight_matrix","Control / stop at 200 PC/FIFO/pattern combinations",test_stop_inflight_matrix},
 {"restart_inflight_matrix","Control / restart at 50 PC/FIFO combinations without glitches",test_restart_inflight_matrix},
 {"restart_preserves_repeat","Control / restart retains configured repetition",test_restart_repeating},
 {"scheduled_restart_cancellation","Control / discard old work at scheduled firing, not at arming",test_scheduled_restart_discards_old},
 {"repeated_stop_start","Control / fifty stop/start cycles with entry zero high",test_stop_then_start},
 {"completion_vs_consumption","Control / FIFO-empty does not mean final entry finished",test_completion_not_fifo_empty},
 {"stop_when_ready","Control / stop after natural completion still applies entry zero",test_stop_after_ready},
 {"full_wire_upload","UART / full 256-entry upload with asymmetric core scheduling",test_full_wire_upload},
 {"debug_reply_wire_order","UART / complete debug dump before next response",test_debug_wire_order},
 {"core0_foreground","UART / actual foreground loop, no interrupt reentrancy",test_core0_foreground},
 {"isolated_reply_poll","UART / response without any additional incoming byte",test_idle_response_poll},
 {"tx_backpressure","UART / paused TX permits fragmented RX progress",test_tx_backpressure},
 {"rx_backpressure","UART / retain command while command queue is full",test_rx_backpressure},
 {"response_backpressure","UART / retain replies while response queue is full",test_response_backpressure},
 {"serial_interleavings","UART / fragmented bytes and varied core-step schedules",test_serial_interleavings},
 {"timing_refill_four_ticks","Timing model / isolated four-tick entry, four-tick service interval",test_refill_four},
 {"timing_refill_one_tick","Timing model / isolated one-tick entry, four-tick service interval",test_refill_one},
 {"packet_lengths","Protocol / packet lengths and version reply",test_protocol_lengths},
 {"mask_duration_boundaries","Protocol / all masks, addresses and duration boundaries",test_masks_durations},
 {"camera_packet_indices","Protocol / clock framing and invalid indices",test_camera},
 {"piv_network_order","Protocol / PIV decoding and enable no-op",test_piv},
 {"pin_source_readback","Protocol / default, configured and unsupported sources",test_pin_sources},
 {"live_pin_routing","Playback / remap queued and held patterns at the correct boundary",test_live_pin_routing},
 {"one_hz_routing","Outputs / shared timer, inversion and immediate override",test_one_hz_routing},
 {"constant_pin_sources","Outputs / immediate On and Off overrides, inversion and release",test_constant_pin_sources},
 {"pianola_status_sources","Outputs / Running and Pending through scheduling, repeating and stop",test_pianola_status_sources},
 {"led_routing","Outputs / Board LED running, external LEDs off and remappable",test_led_routing},
 {"time_wrap_properties","Scheduling / 20,000 seeded modular-clock cases",test_time_arithmetic},
 {"current_time_reply","Protocol / sampled time reply",test_reply_time},
 {"scheduled_wrap_start","Scheduling / normal lead time across wrap",test_scheduled},
 {"replace_pending","Scheduling / replace an armed start",test_replace_pending},
 {"reject_cancels_pending","Scheduling / reject past request and cancel previous start",test_reject_cancels},
 {"missed_deadline","Scheduling / fire on the first poll after the deadline across wrap",test_missed_deadline},
 {"reject_preserves_playback","Scheduling / reject a restart while active playback continues",test_rejected_restart_preserves_playback},
 {"full_sequence","Playback / all 256 entries in order",test_one_shot},
 {"zero_duration","Playback / zero ticks encode as the minimum one tick",test_zero_duration},
 {"excluded_sentinel","Playback / honour final entry despite host sentinel",test_sentinel},
 {"repeat_boundary_matrix","Playback / short/full sequences and repeat-address boundaries",test_repeat_boundaries},
 {"undefined_opcodes","Protocol / undefined opcodes preserve following packet",test_unknown_commands},
 {"scheduled_start_matrix","Scheduling / exact deadlines at signed and wrap boundaries",test_scheduling_matrix},
 {"repeat_section","Playback / prefix then nonzero repeat section",test_repeat},
 {"single_entry_repeat","Playback / bounded sampling of an endless one-entry loop",test_single_repeat},
 {"disable_repeat","Playback / disabling an already active loop",test_repeat_off},
 {"invalid_repeat_boundaries","Safety / invalid repeat at final+1 and 255 across sequence sizes",test_invalid_repeat_boundaries},
 {"repeat_upload_order","Playback / repeat address before final position is preserved",test_repeat_before_final_upload},
 {"live_repeat_invalidation","Safety / invalid repeat edit finishes current pass",test_live_repeat_range_invalidation},
 {"live_final_shrink","Safety / shrinking final while repeating does not strand playback",test_shrink_final_during_repeat},
 {"repair_repeat_range","Playback / repair invalid range before the boundary",test_repair_repeat_range_before_boundary},
 {"invalid_repeat_range","Safety / repeat position beyond final entry",test_invalid_repeat},
 {"stop_pending","Control / stop cancels pending start",test_stop_pending},
 {"stop_queued","Control / stop cancels queued successor",test_stop_queued},
 {"restart_running","Control / restart discards old queued work",test_restart},
 {"hard_reset","Control / hard reset clears playback and configuration",test_hardreset},
 {"uart_core_ownership","Concurrency / UART accessed only by core 0",test_uart_owner},
 {"upload_burst","Transport / serialized no-response command burst",test_queue_burst},
 {"fragmented_packet","Transport / partial packet must not block service",test_partial_packet},
 {"debug_empty_padding","Diagnostics / deterministic empty dump padding",test_debug_dump},
 {"debug_complete_record","Diagnostics / retain complete logged response",test_debug_record},
 {"gpio_configuration","Hardware configuration / valid ranges and precise GPIO ownership",test_configuration},
 {"live_changes_seeded","Safety / 2,000 seeded live edits, stops and queries",test_live_mutation},
 {"production_files","Use cases / production files with exposure substitution",test_workloads},
 {"calibration_sequence","Use cases / calibration and excluded sentinel",test_calibration},
 {"self_trigger_sequence","Use cases / representative self-trigger repetition",test_self_trigger},
};
int main(int argc,char **argv) {
    if(argc==2 && !strcmp(argv[1],"--list")) {for(unsigned i=0;i<count_of(tests);i++) printf("%s\t%s\n",tests[i].name,tests[i].area);return 0;}
    if(argc!=2) {fprintf(stderr,"Pass a test name or --list\n");return 2;}
    setup();
    bool repeat_trace = !strcmp(argv[1],"--repeat-trace");
    if(repeat_trace || !strcmp(argv[1],"--trace")) {
        entry(0,1,1); entry(1,3,4); entry(2,2,0); SEND(2,2);
        if(repeat_trace) { SEND(3,1); SEND(4,1); }
        run();
        expect_next(1,1,"Trace first entry");
        expect_next(3,4,"Trace second entry");
        expect_next(2,0,"Trace final entry");
        if(repeat_trace) {
            expect_next(3,4,"Trace repeat from entry one");
            expect_next(2,0,"Trace repeated final entry");
        }
        printf("{\"words\":[");
        for(unsigned i=0;i<submitted_count;i++) printf("%s%u",i?",":"",submitted[i]);
        printf("]}\n");return 0;
    }
    for(unsigned i=0;i<count_of(tests);i++) if(!strcmp(argv[1],tests[i].name)) {tests[i].fn();return 0;}
    fprintf(stderr,"Unknown test: %s\n",argv[1]);return 2;
}

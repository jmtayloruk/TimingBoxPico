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
static void setup(void) {
    queue_init(&commandQ,32,commandQDepth); queue_init(&responseQ,32,responseQDepth); queue_init(&dumpDebugQ,32,32);
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
    /* Independent physical wiring expectation, not remapPinMask(). */
    const unsigned positions[]={0,9,12,13,23};uint32_t result=0;
    for(unsigned b=0;b<5;b++) if(bits&(1u<<b)) result|=1u<<positions[b];return result;
}
static void expect_next(unsigned bits,uint32_t duration,const char *where) {
    for(unsigned i=0;i<8 && fifo_level<2;i++) step();
    CHECK(consume_entry(),"%s: no complete next entry (FIFO has %u words)",where,fifo_level);
    CHECK(active_duration==duration && active_mask==expected_mask(bits),
        "%s: expected (duration=%u, mask=0x%08x), got (%u, 0x%08x)",
        where,duration,expected_mask(bits),active_duration,active_mask);
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
        CHECK((uint32_t)pianola[b]==d && (pianola[b]>>32)==expected_mask(b),"Entry %u: wrong mask/duration decoding",b);
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
    for(unsigned p=0;p<256;p++) {
        SEND(9,p,28,1);unsigned before=tx_count;SEND(10,p);
        CHECK(tx_count==before+2 && tx[before]==(p<5?p:29) && tx[before+1]==0,"Pin %u: fixed-source readback is inconsistent",p);
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
            CHECK(shouldRunPianolaSoon(now,target)==(d[j]<=3),"Firing-window boundary failed at delta=%u",d[j]);
        }
    }
}
static void test_reply_time(void) {
    fake_time=0x123456;SEND(8);CHECK(tx_count==3 && tx[0]==0x12 && tx[1]==0x34 && tx[2]==0x56,"Current-time reply must contain the sampled 24-bit time in big-endian order");
}
static void test_scheduled(void) {
    entry(0,1,4);SEND(2,0);fake_time=0xFFFF00;
    SEND(6,0,3,0);CHECK(pianolaStatus==PENDING && tx[0]==1,"Scheduled start a few ms ahead across wrap must be accepted");
    fake_time=0x2FC;step();CHECK(submitted_count==0,"Scheduled start executed before the advance window");
    fake_time=0x2FD;step();expect_next(1,4,"Scheduled start across wrap");
}
static void test_replace_pending(void) {
    entry(0,1,4);fake_time=100;step();SEND(6,0,4,0);SEND(6,0,8,0);
    fake_time=1024;step();CHECK(submitted_count==0,"Replacing a pending start left the old start armed");
    fake_time=2048;step();expect_next(1,4,"Replacement pending start");
}
static void test_reject_cancels(void) {
    fake_time=100;step();SEND(6,0,4,0);SEND(6,0,0,99);
    CHECK(pianolaStatus!=PENDING,"Rejected past start did not cancel the previously pending start (XMOS contract)");
}
static void test_missed_deadline(void) {
    entry(0,1,4);fake_time=100;step();SEND(6,0,4,0);
    fake_time=1020;step();fake_time=1025;step();
    CHECK(submitted_count>=2,"Polling skipped the firing window: accepted sequence remains pending after its deadline");
}
static void test_one_shot(void) {
    for(unsigned i=0;i<256;i++) entry(i,i,4+i);
    SEND(2,255);run();
    for(unsigned i=0;i<256;i++) expect_next(i,4+i,"256-entry one-shot");
    for(unsigned i=0;i<8;i++) step();CHECK(fifo_level==0 && submitted_count==512,"One-shot overran final address or duplicated entries");
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
        for(unsigned delta=10;delta>3;delta--) {fake_time=(target-delta)&0xFFFFFF;step();CHECK(!submitted_count,"Start occurred too early in scheduling matrix");}
        fake_time=(target-3)&0xFFFFFF;step();expect_next(1,4,"Matrix scheduled first entry");expect_next(2,8,"Matrix scheduled second entry");
    }
}
static void test_repeat_off(void) {
    entry(0,1,4);SEND(2,0);SEND(4,1);run();
    for(unsigned i=0;i<4;i++) {expect_next(1,4,"Establish repeating state");step();}
    SEND(4,0);
    for(unsigned i=0;i<12;i++) {if(fifo_level>=2) consume_entry();step();}
    CHECK(pianolaStatus!=REPEATING && fifo_level==0,"Disabling repetition did not finish the loop; still repeating after bounded drain");
}
static void test_invalid_repeat(void) {
    entry(0,1,4);SEND(2,0);SEND(3,255);SEND(4,1);run();expect_next(1,4,"Invalid-repeat first entry");
    for(unsigned i=0;i<12;i++) step();
    CHECK(pianolaStatus!=RUNNING && pianolaStatus!=REPEATING,"Repeat address beyond final entry leaves firmware stuck in active state");
}
static void test_stop_pending(void) {
    fake_time=100;step();SEND(6,0,4,0);SEND(7);fake_time=1024;step();CHECK(fifo_level==0 && pianolaStatus!=PENDING,"Stop failed to cancel scheduled start");
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
    CHECK(!(active && active_duration==1000 && fifo_level>=2 && fifo[1]==expected_mask(2)),"Restart left old active/queued work ahead of new entry zero");
    expect_next(1,1000,"Restart must begin at entry zero");
}
static void test_hardreset(void) {
    SEND(4,1);SEND(3,99);SEND(0xFF);
    CHECK(pianolaPosition==0 && pianolaFinalPosition==0 && pianolaRepeatAddr==0 && !pianolaRepeatFlag && !pianolaFireTime,"Hard reset retained repeat/pending configuration");
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
    uint32_t mask=(1u<<2)|(1u<<11)|(1u<<14)|(1u<<15)|(1u<<25);
    // Execute the real startup, including its actual configuration arguments.
    // Stop before either infinite core loop starts; no startup copy lives here.
    trap_enabled=true;
    if(setjmp(trap)==0) firmware_main();
    trap_enabled=false;
    CHECK(!strcmp(trap_reason,"firmware startup reached core-1 launch"),"Startup failed before launching core 1: %s",trap_reason);
    CHECK(output_base==2 && output_count==24,"PIO OUT mapping does not span all five outputs");
    CHECK(gpio_directions==(mask|(1u<<26)|(1u<<27)) && gpio_functions==mask && !(gpio_levels&mask),"Initialisation drove/muxed unintended GPIOs or failed to initialise low");
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
    for(unsigned i=0;i<256;i++) CHECK((uint32_t)pianola[i]==4+i && (pianola[i]>>32)==expected_mask(i),"Full wire upload corrupted entry %u",i);
    CHECK(rx_pos==rx_count && pianolaFinalPosition==255 && tx_count==2 && tx[0]==6 && tx[1]==6,"Full upload lost end marker or following reply");
}
static void test_debug_wire_order(void) {
    const unsigned char commands[]={0xFE,0xFD};receive_bytes(commands,2);service();step();step();
    for(unsigned i=0;i<80;i++) service();
    CHECK(tx_count==1026 && tx[1024]==6 && tx[1025]==6,"Debug dump and subsequent reply interleaved or were lost");
    CHECK(!uart_context_errors,"Debug request caused UART access from core 1");
}

// Compare sequence-controlled GPIOs only; the separate status LEDs legitimately
// change when RUNNING/READY changes and are not part of the frozen pattern.
static uint32_t output_levels(void) {
    return gpio_levels & ((1u<<2)|(1u<<11)|(1u<<14)|(1u<<15)|(1u<<25));
}
static void test_stop_freezes_entry_zero(void) {
    entry(0,17,0xFFFFFF);entry(1,2,1000);entry(2,4,1000);SEND(2,2);run();
    expect_next(17,0xFFFFFF,"Entry-zero start");step();expect_next(2,1000,"Different active pattern");step();
    CHECK(fifo_level==2,"Expected queued successor before stop");
    SEND(7);
    uint32_t expected=expected_mask(17)<<2;
    CHECK(output_levels()==expected,"Stop must apply nonzero entry-zero pattern, expected GPIO=0x%x got 0x%x",expected,gpio_levels);
    CHECK(!active && !sm_enabled[0] && !fifo_level,"Stop retained active/queued work or left output SM enabled");
    CHECK(pianolaPosition==0 && pianolaStatus==READY && !pianolaFireTime && !pianolaRepeatFlag,"Stop did not rewind/cancel pending/repeat state");
    for(unsigned i=0;i<100;i++) {fake_time+=100;step();CHECK(!consume_entry(),"Stopped driver consumed a new entry");}
    CHECK(output_levels()==expected,"Stopped entry-zero pattern was not held indefinitely");
    CHECK(!clock_control_changes && sm_enabled[1],"Stop modified the independent clock state machine");
}
static void test_stop_inflight_matrix(void) {
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
        CHECK(output_levels()==(expected_mask(masks[m])<<2),"Stop wrong entry-zero pattern for mask %u, PC %u, FIFO %u",masks[m],pc,words);
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
        CHECK(fifo_level==2 && fifo[0]==1000 && fifo[1]==expected_mask(1),"Restart retained old FIFO words");
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
    CHECK(!active && fifo_level==2 && fifo[0]==1000 && fifo[1]==expected_mask(1),"Scheduled firing retained old active/queued work");
    expect_next(1,1000,"Scheduled restart entry zero");
}
static void test_stop_then_start(void) {
    entry(0,16,4);entry(1,1,8);SEND(2,1);
    for(unsigned cycle=0;cycle<50;cycle++) {
        run();expect_next(16,4,"Repeated start");step();SEND(7);
        CHECK(!sm_enabled[0] && output_levels()==(1u<<25),"Repeated stop failed to freeze LED-high entry zero");
    }
}
static void test_completion_not_fifo_empty(void) {
    entry(0,1,1000);SEND(2,0);run();expect_next(1,1000,"Final entry in flight");step();
    CHECK(pianolaStatus==RUNNING,"FIFO consumption was incorrectly treated as final-entry completion");
    active=false;driver_pc=driver_wait_pc;step();
    CHECK(pianolaStatus==READY,"Finished final entry did not transition to READY at the wait loop");
}
static void test_stop_after_ready(void) {
    entry(0,16,4);entry(1,1,8);SEND(2,1);run();
    expect_next(16,4,"Final-state setup 0");expect_next(1,8,"Final-state setup 1");
    active=false;driver_pc=driver_wait_pc;step();CHECK(pianolaStatus==READY,"Expected ready state");
    SEND(7);CHECK(output_levels()==(1u<<25) && !sm_enabled[0],"Stop while already ready did not apply entry-zero pattern");
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
 {"fixed_source_readback","Protocol / documented fixed-source limitation",test_pin_sources},
 {"time_wrap_properties","Scheduling / 20,000 seeded modular-clock cases",test_time_arithmetic},
 {"current_time_reply","Protocol / sampled time reply",test_reply_time},
 {"scheduled_wrap_start","Scheduling / normal lead time across wrap",test_scheduled},
 {"replace_pending","Scheduling / replace an armed start",test_replace_pending},
 {"reject_cancels_pending","Scheduling / reject past request and cancel previous start",test_reject_cancels},
 {"missed_start_window","Scheduling / accepted start must not remain pending indefinitely",test_missed_deadline},
 {"full_sequence","Playback / all 256 entries in order",test_one_shot},
 {"excluded_sentinel","Playback / honour final entry despite host sentinel",test_sentinel},
 {"repeat_boundary_matrix","Playback / short/full sequences and repeat-address boundaries",test_repeat_boundaries},
 {"undefined_opcodes","Protocol / undefined opcodes preserve following packet",test_unknown_commands},
 {"scheduled_start_matrix","Scheduling / start windows at signed and wrap boundaries",test_scheduling_matrix},
 {"repeat_section","Playback / prefix then nonzero repeat section",test_repeat},
 {"single_entry_repeat","Playback / bounded sampling of an endless one-entry loop",test_single_repeat},
 {"disable_repeat","Playback / disabling an already active loop",test_repeat_off},
 {"invalid_repeat_range","Safety / repeat position beyond final entry",test_invalid_repeat},
 {"stop_pending","Control / stop cancels pending start",test_stop_pending},
 {"stop_queued","Control / stop cancels queued successor",test_stop_queued},
 {"restart_running","Control / restart discards old queued work",test_restart},
 {"hard_reset","Control / reset clears repeat and pending configuration",test_hardreset},
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
    if(!strcmp(argv[1],"--trace")) {
        entry(0,1,4); entry(1,1,4); SEND(2,1); run();
        expect_next(1,4,"Trace first entry");expect_next(1,4,"Trace second entry");
        printf("{\"words\":[");
        for(unsigned i=0;i<submitted_count;i++) printf("%s%u",i?",":"",submitted[i]);
        printf("]}\n");return 0;
    }
    for(unsigned i=0;i<count_of(tests);i++) if(!strcmp(argv[1],tests[i].name)) {tests[i].fn();return 0;}
    fprintf(stderr,"Unknown test: %s\n",argv[1]);return 2;
}

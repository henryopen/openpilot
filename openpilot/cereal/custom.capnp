using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

using Car = import "/car.capnp";

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# Every target the front radar reports, where radarTracks carries only the one radard is
# allowed to follow. A guardrail scanned along its length looks like a car keeping station
# ahead, so handing the whole list to radard measured worse than vision alone; the display
# and anything that watches the lanes beside us still wants all of it.
struct RadarTracksSP @0x81c2f05a394cf4af {
  points @0 :List(Car.RadarData.RadarPoint);
}

# What actually set the longitudinal accel this frame. The plan source alone cannot say:
# stop_for_lights borrows the e2e slot outside experimental mode, and the curve limiter
# never appears at all - it caps cruise from inside. Display only.
struct LongitudinalPlanSP @0xaedffd8f31e7b55d {
  reason @0 :Reason;

  enum Reason {
    cruise @0;
    lead @1;
    stopLight @2;
    curve @3;
    e2e @4;
    weakLead @5;
  }
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}

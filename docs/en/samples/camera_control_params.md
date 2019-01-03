# Camera control parameters API {#camera_control_params}

## Open or close auto exposure

```
/** Auto-exposure enabled or not  default enabled*/
bool AutoExposureControl(bool enable);    see "camera.h"
```

## Open or close auto white balance

```
/** Auto-white-balance enabled or not  default enabled*/
bool AutoWhiteBalanceControl(bool enable);    see "camera.h"
```

## Set infrared(IR) intensity

```
/** set infrared(IR) intensity [0, 10] default 4*/
void SetIRIntensity(const std::uint16_t &value);     see "camera.h"
```

## Set global gain

Note:: You have to close auto exposure first.

```
/** Set global gain [1 - 16]↩
 * value -- global gain value↩
 * */↩
void SetGlobalGain(const float &value);    see "camera.h"
```

## Set the exposure time

Note:: You have to close auto exposure first.

```
/** Set exposure time [1ms - 2000ms]↩
 * value -- exposure time value↩
 * */↩
void SetExposureTime(const float &value);    see "camera.h"
```


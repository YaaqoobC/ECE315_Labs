# Notes for report
## Problems
- mention turning LED completly off
- Passing NULL to the Queue for the struct to hold the data
- adding zero timeout for receiving from from queue **FIXED** a problem we encountered where we had a timeout and the SSD right side flickered too much

## Design
- passing a struct instead of bytes

## Testing
### SSD
- same number twice
- 2 different nums
- 1 num by itself at very beginning
- holding one down (only does one value -> intended behaviour)
- press one number, wait a few secs, press another

### RGB LED
- go all the way up to max vals
- all the way down to lowest xOnDelay = 0 (RGB was off -> intended behaviour)
- middle buttons, nothing happened
- up and down buttons alternating

- RGB into SSD inputs to make sure no collisions across GPIO's

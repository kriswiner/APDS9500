# APDS9500

Collection of Arduino sketches for Broadcom's APDS9500 gesture sensor and IR camera.

![APDS9500](https://cloud.githubusercontent.com/assets/6698410/26532697/728712f0-43bd-11e7-888e-a0cf049c16e7.jpg)

So far I have gotten the device to respond to gestures via polling and interrupts but the majority of gestures it detects are waves, and occasionally up, down, forward and backward, etc. It doesn't seem to be too discriminating, treating everything it detects as a wave. This just means the thresholds are not set properly. Work in progress...

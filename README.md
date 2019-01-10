PWP3D
=====

This is a cmake version of the PWP3D. The original VS version is availabled at:

http://www.robots.ox.ac.uk/~victor/code.html

For more detail of the paper, see:

See original paper from:
PWP3D: Real-time Segmentation and Tracking of 3D Objects
Victor Adrian Prisacariu, Ian Reid

## Demo

Run the shuttle example by executing the following command

```
Application/PWP3DAPP 0.2 -2 24 20 180 -70 0.2 0.5 10
```

First 3 parameters indicate initial starting position, next 3 initial rotation angle in degrees and final 3 set the gradient step size: first for rotation on all 3 axises, second for translation on x-y axis and third for translation on z axis, or scale.
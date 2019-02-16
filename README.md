PWP3D
=====

This is a cmake version of the PWP3D. The original VS version is availabled at:

http://www.robots.ox.ac.uk/~victor/code.html

For more detail of the paper, see:

See original paper from:
PWP3D: Real-time Segmentation and Tracking of 3D Objects
Victor Adrian Prisacariu, Ian Reid

## Demo

### Still image

Run the following command for a Shuttle convergence example similar to the one provided by the authors

```
PWP3DAPP Images/shuttle.jpg Masks/shuttle.png Models/shuttle.obj 1 0 -2 24 20 180 -70 0.1 0.5 10
```

The last 10 numbers define initialization parameters. First is iteration count, next 3 indicate initial starting position, next 3 initial rotation angle in degrees and final 3 set the gradient step size: first for rotation on all 3 axises, second for translation on x-y axis and third for translation on z axis, or scale.

### Sequence of images

Download RGB images of a power drill in [YCB dataset](http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com). Resize the images to 1920x1280 and rename sequentially, padding with leading zeros. Then execute the following command:

```
PWP3DAPP path/to/images_*.jpg Masks/drill_n1.png Models/drill_4k.obj 10 0 0 24 88 192 0 0.1 0.2 2
```

This should produce results similar to the [video](https://www.youtube.com/watch?v=O7tr2cwzhsQ). Note that the first argument should actually be expanded by the shell globbing rules. You can also list multiple images manually instead.

To test your own images, repeat the steps above for input files. Additionally you'll need the following:

* An alpha-mask of the object in the first image in the set for background / foreground histogram initialization.
* 3D mesh model of the object. If it's a super high-poly laser-scanned model, better simplify it in [Meshlab](//github.com/cnr-isti-vclab/meshlab).
* Intrinsic camera calibration parameters. Create your own with this OpenCV [tutorial](https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html), or simply try generic first.

After this, play with initialization pose. The virtual object should clearly overlap the real one and be rotated less than 90° away on all axises. Set initial step size. Keep in mind that it's dependent on object's distance to the camera, so it will decrease logarithmically as the object gets closer and wise-versa.
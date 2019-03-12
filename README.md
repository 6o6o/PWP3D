PWP3D
=====

This is a cmake version of the PWP3D. The original VS version is availabled at:

http://www.robots.ox.ac.uk/~victor/code.html

For more detail of the paper, see:

See original paper from:
PWP3D: Real-time Segmentation and Tracking of 3D Objects
Victor Adrian Prisacariu, Ian Reid

## Contribution

While dealing with the project, the following issues were encountered:

* Limited Z-buffer range, causing objects located beyond to break like [this](https://www.youtube.com/watch?v=JzWpxeS0Y38). Increasing it allows to support larger models without having any accuracy implications. Read more in section 5.1 of the paper.

* Step size depends on distance to the camera, decreasing as the object gets closer and vise-versa. A simple function is provided to dynamically adjust it according to distance difference. Additional parameter `k` specifies the rate of change, so that starting distance of `24` with step size of `0.1` would produce this [plot](/Files/Results/step_size.png).

The main challenges are manual initialization and continuous adaptation of the background and foreground histograms. Simply updating them from object pose only causes quicker corruption even under prefect 3D model alignment. More sophisticated technique has been developed recently [[1](https://doi.org/10.1007/s11263-015-0873-2), [2](https://doi.org/10.1109/iccv.2017.23)], but implementing it will take some effort.

## Demo

### Still image

Run the following command for a Shuttle convergence example similar to the one provided by the authors

```
PWP3DAPP Images/shuttle.jpg Masks/shuttle.png Models/shuttle.obj 1 0 -2 24 20 180 -70 0.04 0.1 1
```

The last 10 numbers define initialization parameters. First is iteration count, next 3 indicate starting position, next 3 initial rotation angle in degrees and final 3 set the gradient step size: first for rotation on all 3 axises, second for translation on x-y axis and third for translation on z axis, or scale change.

### Sequence of images

Download RGB images of a power drill in [YCB dataset](http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com). Resize them to 1920x1280 and rename sequentially, padding with leading zeros. Then execute the following command:

```
PWP3DAPP path/to/images_*.jpg Masks/drill_n1.png Models/drill_4k.obj 10 0 0 24 88 192 0 0.1 0.2 2
```

This should produce results similar to the [video](https://www.youtube.com/watch?v=O7tr2cwzhsQ). Note that the first argument actually has to be expanded by the shell globbing rules. You can also list multiple images manually instead.

To test your own images, prepare your files as before. Additionally you'll need the following:

* An alpha-mask of the object in the first image in the set for background / foreground histogram initialization.
* 3D mesh model of the object. If it's a super high-poly laser-scanned model, better simplify it in [Meshlab](//github.com/cnr-isti-vclab/meshlab).
* Intrinsic camera calibration parameters. Create your own with this OpenCV [tutorial](https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html), or simply try generic first. They scale along with resolution, so you only need one file for various image dimensions.

In the source code set your image resolution, path to calibration file and video mask.

Coming up with the right set of parameters is a bit tricky. Usually these are unique per object and can be determined experimentally. First, set the distance to the camera and vary the pose. The virtual object should clearly overlap the real one and be rotated less than 90° away on all axises. Then adjust initial step size relative to the distance, so it would be scaled accordingly falling in the best working range.
# Cpp-Boids-Simulation 

Stable boids simulation in C++ written using standard libraries along with Eigen for linear algebra and SFML as the graphics library. Feel free to tweak some of the parameters in the code, the settings I've got it at are for a few large stable flocks to fly generally in parallel with only occasional large scale interactions.

All of the code you likely care about is in main, everything else was built with Visual Studio's default C++ solution setup.

Using under 625 (a square grid of sides 25) boids is recommended as the application is single threaded. Multithreading/multiprocessing is slower at this amount of boids due to thread overhead. The program is set to 400 boids by default which is a nice compromise between simulation speed and boid count.


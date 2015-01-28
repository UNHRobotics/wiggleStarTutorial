This is a tutorial that will (hopefully) walk you through implementing your first planner for OMPL and getting it run in the app. I will attempt to keep this moderately self-contained, but links will be included to the relevant links on the OMPL website (//TODO: link to ompl website).

#Installation

The steps for installation are pretty thorough on the website, (//TODO: link) but are included here just to keep things self-contained. I'll be using a relatively new install of Ubuntu 14.04 LTS.

    sudo apt-get install git libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev libtriangle-dev
    git clone git://github.com/ompl/omplapp
    cd omplapp
    git clone git://github.com/ompl/ompl
    mkdir -p build/Release
    cd build/Release
    cmake ../..
    make installpyplusplus && cmake .
    make update_bindings
    sudo make -j 4 install

If you want, you can now try out the app by running ../../gui/ompl.py. Problem configurations can be found in ../../resources.

#Wiggle**
(*Not actually optimal)

We'll be implementing a planner that simply samples a random control and applies it until it reaches the goal state. The website has a template for making a new planner, just for reference. (//TODO: link) Anyways, let's copy in the files for our simple planner.

    mkdir ../../ompl/src/ompl/control/planners/wiggleStar
    cd ../../ompl/src/ompl/control/planners/wiggleStar
    cp /path/to/WiggleStar.h .
    mkdir src
    cp /path/to/src/WiggleStar.cpp src

The comments should hopefully by sufficient to understand what's going on, but the gist of it is that we only need to implement the solve function (and  constructor, but whatever). We are expanding states by propagating controls from our previous state, and tacking the new state on to the end of a list. Once we find a path to the goal or time runs out, we stick our list of states and controls into the Path class and hand it back to whoever called us. I realize it's a waste of time to keep the states in our own list if we're just putting them into the Path afterwards, but I wanted this to serve as a fuller template than the one OMPL gives, and I didn't want to make things harder for you down the road since you (hopefully).have more complicated data structures where getting the path isn't so straightforward.

#Adding to OMPLapp

Adding to the app actually isn't too bad, once you know what you're doing. The OMPL docs have instructions on how to do this, but they're subtly wrong and a bit out of date so while they're a good reference, I'm going to walk through it myself.

Since we're making a control planner, we'll only be touching the control files. If you're making a geometric planner, you'll only mess with the geometric stuff. First we'll add our file path to `headers_control.txt`:

    cd ../../../../../../
    nano ompl/py-bindings/headers_control.txt

Next we have to add the planner to the bindings generator:

    nano  ompl/py-bindings/generate_bindings.py

Here, we'll add the name of the planner to two arrays so the generation script knows to look for it. We'll look in the `ompl_control_generator_t` class for the `filter_declarations()` function. Again, if you're making a geometric planner, look for `ompl_geometric_generator_t`. In our case (and for this version, 1.0.0) this on line 456. Just stick `'WiggleStar'` at the end there, and then we run

    cd ../../build/Release
    make clean_bindings
    make -j 4 update_bindings

And we should be ready to go!

*Note: This is probably still broken right now. Fixes will come.*

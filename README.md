# Urho3D-1.4-SphericalHarmonicLighting

Spherical Harmonic Lighting implementation written with Urho3D engine version 1.4.

Information
-----------------------------------------------------------------------------------
Spherical Harmonic Lighting based on: 
http://www.research.scea.com/gdc2003/spherical-harmonic-lighting.pdf

This is a simple implementation of the SHL using a single static light and static models.  
This example uses low polygon mesh and may not look like other examples of SHL, so you
may want to replace the models with your own high polygon mesh to see a better diffuse transfer.

Implementation:
1 - diffused unshadowed transfer
2 - shadowed diffuse transfer
3 - diffuse interreflected transfer


To build it in Urho3D 1.4:
This implementation is built as a Urho3D sample.  Copy the source folder into your Urho3D/ folder,
copy the bin folder into your build/bin folder, and add 61_SHLighting in your Source/Samples/CMakeLists.txt.


Licensed under the MIT license
-----------------------------------------------------------------------------------
Copyright (c) 2015 Lumak Software.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
DEALINGS IN THE SOFTWARE.


Screenshots
-----------------------------------------------------------------------------------
Normal lighting:
![alt tag](https://github.com/Lumak/Urho3D-1.4-SphericalHarmonicLighting/blob/master/screenshot/normallighting.jpg)

Diffused unshadowed transfer:
![alt tag](https://github.com/Lumak/Urho3D-1.4-SphericalHarmonicLighting/blob/master/screenshot/unshadowed.jpg)

Shadowed diffuse transfer:
![alt tag](https://github.com/Lumak/Urho3D-1.4-SphericalHarmonicLighting/blob/master/screenshot/shadowed.jpg)

Diffuse interreflected transfer:
![alt tag](https://github.com/Lumak/Urho3D-1.4-SphericalHarmonicLighting/blob/master/screenshot/interreflect.jpg)







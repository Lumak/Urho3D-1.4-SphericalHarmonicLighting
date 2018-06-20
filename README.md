# Urho3D-1.4-SphericalHarmonicLighting

Spherical Harmonic Lighting implementation written with Urho3D engine version 1.4.

Information
-----------------------------------------------------------------------------------
Spherical Harmonic Lighting based on: 
[spherical-harmonic-lighting.pdf](https://github.com/Lumak/Urho3D-1.4-SphericalHarmonicLighting/blob/master/spherical-harmonic-lighting.pdf)

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

License
----
The MIT License (MIT)






# NDTM

This repository consists of complementary material from the article titled "The Normalized Direct Trigonometry Model for the Two-Dimensional Irregular Strip Packing Problem," developed by German Pantoja-Benavides, David Álvarez Martínez, and Francisco Parreño Torres.

In the "Instances" folder, you will find the .txt files of the instances used with the following format:
Strip'sWidth NumberOfTypeOfPieces
Then, for each type of piece, the format is as follows:
NumberOfDuplicates NumberOfConvexParts NumberOfVertices NumberOfValidOrientations
Orientations separated by spaces
List of the index of the vertices that form the convex parts, one line per each convex part
The coordinates of the vertices (x,t) of all vertices that form the irregular piece

In the "Solutions" folder, you will find two subfolders: "Images" and "texFiles." In the former, you will find the visual representation of the solutions. In the latter, you will find the .txt files of the found solutions.

Finally, in the "Code" folder, you will find the source code used to generate the solution files. The source code NDTM has all the models proposed in the paper. The source code DTM is our implementation of the model proposed by Cherri et al. (2016) in the paper titled "Robust mixed-integer linear programming models for the irregular strip packing problem."

Both implementations have the following parameters:
-t indicates the maximum execution time for the optimizer (CPLEX) to solve an instance
-nThreads indicates the number of threads that the optimizer can use
-ins indicates the name of the instance
-area indicates the total area of the pieces of the instances

The implementation of the NDTM has these additional parameters:
-sep indicates how much distance the items must be separated within the strip
-m indicates the model that will execute: -1 for the NDTM1, 0 for the NDTM, 1 for the NDTM_Mod1, and 2 for the NDTM_Mod2

It is worth noting that the executable file must be in the same directory as the "Instances" and "Solutions" folders.

Any questions about this repository should be addressed to Germán Pantoja via e-mail: germanpantojab@gmail.com

# Notes on Box2D and usage of it

Box2D source code here is provided unaltered from commit [a174dbac1bc32e05e790cd867d9e30fb37681d47](https://github.com/erincatto/Box2D/tree/a174dbac1bc32e05e790cd867d9e30fb37681d47) from Box2D original source. 

***We aren't going to use it directly so there's no need to build it***

Instead of using either premake or the cmake script made by [jschmold](https://github.com/erincatto/Box2D/pull/529) and [felaugmar](https://github.com/felaugmar/Box2D/commit/7f84e6b286eb2755220eedbdf178eb418384ecfa), we instead use an amalgamated version of it.

We will use the tool [amalgamate](https://github.com/shrpnsld/amalgamate) by Anton Degtiar, which is provided with [MIT License](https://github.com/shrpnsld/amalgamate/blob/master/LICENSE.txt).

To generate an amalgamated version we run `./amalgamate -I .` on the Box2D directory.

This will make easier to build agsbox2d in environments that require cross-compilation (like Android).

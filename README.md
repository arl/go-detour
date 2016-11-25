# go-detour [![GoDoc](http://img.shields.io/badge/go-documentation-blue.svg?style=flat-square)](http://godoc.org/github.com/aurelien-rainone/go-detour) [![Build Status](https://travis-ci.org/aurelien-rainone/go-detour.svg?branch=master)](https://travis-ci.org/aurelien-rainone/go-detour) [![Coverage Status](https://coveralls.io/repos/github/aurelien-rainone/go-detour/badge.svg?branch=master)](https://coveralls.io/github/aurelien-rainone/go-detour?branch=master)


## What is go-detour?

**[go-detour][1]** is a [Detour][2], an amazing multi-platform library for
navigation mesh path-finding, to the [Go language][3].


## So what is Detour?

From [Detour][2] README:
> **Detour** is a path-finding and spatial reasoning toolkit. You can use any
> navigation mesh with **Detour**, but of course the data generated with
> **Recast** fits perfectly.

> **Detour** offers simple static navigation mesh which is suitable for many
> simple cases, as well as tiled navigation mesh which allows you to plug in
> and out pieces of the mesh. The tiled mesh allows you to create systems where
> you stream new navigation data in and out as the player progresses the level,
> or you may regenerate tiles as the world changes. 


## Credits

**[go-detour][1]** has been ported to the Go language by [Aurélien Rainone][1],
and as such, is not an original work.


**[Detour][2]**, the original work from which current library is a port of, has
been written by Mikko Mononen, and released under the following:
> copyright (c) 2009 Mikko Mononen memon@inside.org.


## License

**[go-detour][1]** is licensed under ZLib license, see [LICENSE][5] for more
information.


[1]: https://github.com/aurelien-rainone/go-detour "go-detour"
[2]: https://github.com/recastnavigation/recastnavigation "Recast & Detour"
[3]: https://github.com/golang/go "The Go language"
[4]: https://github.com/aurelien-rainone
[5]: ./LICENSE "License"

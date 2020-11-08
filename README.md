## First Steps
1. If you have protobuf installed, check which version you use with the command:
```sh
protoc --version
```

1. If you do not , follow the instructions in [C++ Installation - Unix](https://github.com/google/protobuf/tree/master/src).

1. According to its version download the referent in [protobuf/tags](https://github.com/google/protobuf/tags).

1. Extract the downloaded file and go to `/protobuf-3.6.1/src/` *(example version)*.

1. Copy the google folder and paste it into `/ssl-Client/net`

1. Go to folder `/ssl-Client/net/pb/proto/` and run:
```sh
sh compile.sh
```

*Obs: when building the program, if something referring to `google/protobuf/stubs/common.h` has any errors, it will probably be an incompatibility version problem with protobuf (`net/google`)*.

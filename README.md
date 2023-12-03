# ACL Unity

This is a Unity Native Plugin which wraps a subset of
[ACL](https://github.com/nfrechette/acl). Its primary purpose is for use in
DOTS-based projects.

## About ACL

Animation Compression Library, or ACL, is a C++11 library written by Nicholas
Frechette and has been utilized in multiple AAA titles. It uses variable
bit-rate compression of fixed-rate sampled poses for fast and highly accurate
compression and decompression of both short and long clips. It is
hierarchy-aware, further increasing its accuracy. It supports many platforms and
can be compiled to utilize various levels of SIMD instructions.

## About this Unity Plugin

Nicholas Frechette maintains an official Unreal Engine plugin. However, due to
the closed-source nature of traditional Unity, he does not develop an equivalent
for Unity.

This is an unofficial plugin for Unity primarily targeting DOTS-based projects.
I (Dreaming381) wrote this wrapper for the needs of the [Latios
Framework](https://github.com/Dreaming381/Latios-Framework). But feel free to
use it or improve it as you see fit.

## Why ACL and DOTS?

Traditionally, Unity uses curves for animation. It achieves performance by
caching parts of the curve evaluation for use in the next evaluation. However,
this approach doesn’t work as well in a DOTS project, where statelessness is
preferred. ACL uses a technique that does not require caches to persist between
frames in order to achieve great performance. It is also developed with a
data-oriented mindset and API, making integration relatively straightforward.

If you are considering using this in your own DOTS-based animation solution,
consider storing compressed tracks in `BlobArray`s with forced 16-byte
alignment, and calling directly into this plugin from Burst jobs. You will also
want to borrow the ACL_Unity directory from the Latios Framework which contains
proper C\# wrappers with safety checks.

## Debugging with Latios Framework

Beginning with Latios framework 0.5.2, ACL Unity uses GitHub actions to generate
build artifacts. These artifacts contain the binaries that were copied to the
Latios Framework, as well as debug symbols and human-readable assembly text
files.

## Platforms Supported Out-of-the-Box

The following platforms are supported out-of-the-box with binaries generated via
GitHub Actions:

-   Windows
-   Mac OS (both x86 and ARM)
-   Linux (x86 only)
-   Android (ARM 32-bit and 64-bit)

## Adding Platform Support for the Latios Framework

Disclaimer: I am not very good with DevOps and am pretty happy I even got this
project this far.

Adding support for a new platform should not require any C++ code changes.
Instead, you will need to modify the top-level CMakeLists.txt to detect your
target platform and specify the correct compiler flags for the different
supported CPU architectures. The current CMakeLists.txt assumes your target
platform is the same device that you are compiling for.

Once you have successfully compiled the library, import it into Unity as a
native plugin. Depending on your target platform, you may need to modify the
Kinemation/ACL_Unity C\# files to load the correct version of the compiled
library plugin for a given architecture. Note that when Burst is disabled, the
code will always use the base version of the plugin with no extra CPU features.

Lastly, if your target platform is not constrained by NDA, you can support it in
GitHub actions by modifying .github/workflows/acl-cross-platform-build.yml and
adding a dedicated job with your platform. Every time you push changes, the jobs
will run and you will be able to download and inspect artifacts. Test the
artifacts to ensure they work correctly. Then create a pull request.

## How Fast is It?

Fast!

And not like “fast enough for my personal use” fast.

More like “I doubt Unity’s engineers will ever beat this” fast.

Most animation sampling solutions (including Unity’s) require a keyframe search
to find two keyframes the time value falls between. ACL doesn’t need to do this
and instead looks up compressed byte offsets in a few instructions.

In addition, Burst support for native plugins is truly “native”. Performance of
calling into this plugin from Burst jobs is on par with performance using ACL
directly in C++ projects. Burst users get the full performance ACL has to offer.

So yeah. Fast!

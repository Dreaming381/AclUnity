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
consider storing compressed tracks in BlobArrays with forced 16-byte alignment,
and calling directly into this plugin from Burst jobs.

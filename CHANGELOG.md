# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]


## [0.2.0] - 2021-03-22


### Addded

- `std` feature that adds `std::error::Error` trait implementations to error
  types.
- You may license the crate either under the MIT or Apache 2.0 license, your
  choice.
- Added implementations of common traits (such as `Clone`, `Debug`, etc.) to
  types.

### Changed

- Renamed `mh_z19c::frame::Error` to `ValidateFrameError`.
- Changed `Display` implementation of error types to produce lowercase messages
  without trailing punctuation as suggested by the Rust API guidelines.


## [0.1.1] - 2021-03-13

### Added

- Changelog

### Fixed

- Typo in the main example.


## [0.1.0] - 2021-03-13

Initial release.

[Unreleased]: https://github.com/olivierlacan/keep-a-changelog/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/olivierlacan/keep-a-changelog/compare/v0.1.0...v0.2.0
[0.1.1]: https://github.com/olivierlacan/keep-a-changelog/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/olivierlacan/keep-a-changelog/releases/tag/v0.1.0


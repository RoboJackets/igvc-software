# Project Title

*Issue #Number*
576

**Author:**
- Vivek Mhatre

## The Problem

Currently we have difficulty asserting whether a subscriber has received a message. Without race conditions, testing is hard to do.

## Proposed Solution

- To solve this problem I will create a testing library that abstracts the testing file. This way, we will be able to reuse the same tests for other nodes and possibly incorporate the directory into other projects.
- First abstract the current testing file into testing directory.
- Create tests to check if testing library works.

## Questions & Research
- How do I write tests?
- Oswin's comments on [issue 575]{https://github.com/RoboJackets/igvc-software/issues/575)


## Overall Scope

### Affected Packages

- igvc_platform/src/tests

### Schedule

Subtask 1 (02/23/2020): Abstract test into test directory

Subtask 2 (03/02/2020): Write own tests for test directory

Code Review (03/07/2020): Should be done by now and ready for review. Hopefully I can be done sooner.

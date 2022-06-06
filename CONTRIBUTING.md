# Contributing to HSolo

Welcome, and thank you for wanting to contribute to the HSolo project. The procedure to contribute is of the following:

## Create a GitHub Issue
Navigate to HSolo's GitHub Issues page and create a new issue. The issue can be used for any number of things &mdash; reporting a bug, suggesting an enhancement, posing a
question, etc. Please be as descriptive as possible, this will allow us to better understand the issue. 

## Work an Issue

When work is ready to commence on an issue, the workflow to use is the following:

### Fork HSolo

* If you have not already done so, create a fork of HSolo on GitHub under your username.
* Clone your fork of HSolo with `git clone git@github.com:<username>/HSolo`.
* Each time you clone your fork, `git remote add upstream git@github.com:hsolo/HSolo` to add the original HSolo repository as the `upstream` remote.

### Update the Main Development Branches

To keep your `master` and `develop` branches up-to-date with those from `upstream`:

* `git fetch --all`
* `git checkout master`
* `git merge upstream/master`
* `git push origin master`
* `git checkout develop`
* `git merge upstream/develop`
* `git push origin develop`

You want to do this before starting work on a new feature branch.

> **Note:**  Updating `master` is not strictly necessary, as all development work is done off of `develop`.

### Create a Feature Branch

Create a local branch off of `develop` on which to make your changes:

* `git checkout develop`
* `git checkout -b <branchName>`

`<branchName>` can be whatever you like, though we have some recommendations:
* Include the issue number in it in some way, for instance, `123-<restOfBranchName>`, or `<restOfBranchName>-123`.
* Make the branch name descriptive; that is, avoid `fixSomeStuff`, `performanceTweaks`, and generic names along those lines.
* To indicate your branch is intended solely for your own use, preface the branch name with your username, as in `<username>/<restOfBranchName>`.

### Make Your Changes

Do whatever work is necessary to address the issue you're tackling, breaking your work into logical, compilable commits.  Feel free to commit small chunks early and often in your local repository and then use `git rebase -i` to reorganize your commits before sharing.  Make sure the commit messages you will be sharing reference the appropriate GitHub issue numbers.

### Update Your Branch

While working on your feature in your local `<branchName>` branch, other commits will likely make it into the real HSolo `develop` branch.  There are a variety of ways to merge these changes into your local feature branch.  One possibility is

* `git checkout <branchName>`
* `git fetch --all`
* `git merge upstream/develop`

though there are others that are equally valid.

### Create a Pull Request

When your changes are ready to be integrated into HSolo's `develop` branch:

* Push your local feature branch up to your fork with `git push -u origin <branchName>`.
* Navigate to your fork of HSolo on GitHub and create a new pull request:
  * Be sure you choose:
    * base fork:  `hsolo/HSolo`
    * base:  `develop`
    * head fork:  `<username>/HSolo`
    * compare:  `<branchName>`
  * On the new pull request creation page, please give as much information in the *Description*  to allow us to review and approve the issue as soon as practicable.

### Feedback

At this point you'll enter into a stage where you and various HSolo developers will iterate back and forth until your changes are in an acceptable state and can be merged in.  If you need to make changes to your pull request, make additional commits on your `<branchName>` branch and push them up to your fork.  Make sure you don't delete your remote feature branch or your fork of HSolo before your pull request has been merged.
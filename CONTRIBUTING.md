# Contributing to ARLab

First off, thanks for taking the time to contribute! ❤️
I'm really glad you're reading this, because we need volunteer developers and many more to help this project come to fruition.

Please read our [Code of Conduct](./CODE_OF_CONDUCT.md) to keep our community approachable and respectable.

In this guide you will get an overview of the contribution workflow from opening an issue, creating a MR, reviewing, and merging the MR.

## Issues

### Create new issue

If you spot a problem or have an idea to work on, search if an issue regarding this already exists. If a related issue doesn't exist, you can open a new issue using a relevant issue form.

You can open an issue [in ARLab Issues](https://github.com/una-auxme/arlab/issues/new/choose).

If you're reporting a **bug**, please include the output of your used tools like with the following commands:

```sh
uname -a
rustc --version
ls -la
git status
```

### Solve an issue

Scan through our existing issues to find one that interests you. You can narrow down the search using labels as filters. See Labels for more information. As a general rule, we don’t assign issues to anyone. If you find an issue to work on, you are welcome to open a MR with a fix.

### Kanban based

We work in a **Kanban** based development. This means:

1. first create an issue for every ticket/bug/enhancement in our Git(Hub), label it accordingly and add a description so somebody else can work on it too (even if you soly do it, somebody else need this for review)
2. assign an open ticket to yourself. Create a MR and Branch you can work on. Push all changes on this Branch and set the ticket to "to review" if ready
3. all work regarding a single issue is done in a single MR/Branch
4. a issue should be reviewed by at least one other member of the community, best case two. After approving the MR, the ticket can be labeled "ready to merge".
5. MR onto main can only be applied by higher-level members of the community to keep control over breaking changes and organize everything around.

### Merge Request Workflow

Once you open a Merge Request, it may be reviewed or labeled (or both) until
the maintainers accept your change. Please be patient, it may take some time
for this to happen!

To let other maintainers know to review your MR/Issue, add the "to review" tag to the Issue in the Issueboard.

### Sample of a full workflow

1. create issue (e.g. "add CONTRIBUTING.md", the issue is automatically assigned a number, e.g. #1)
2. choose an issue, assign yourself to it, set label to "doing"
3. create a merge request on the issue (the MR is automatically assigned a number, e.g. !5)
4. checkout to the created branch on your dev machine, e.g. `git checkout 1-add-contributing-md`
5. work on the issue...
6. add all changes, e.g. `git add CONTRIBUTING.md`
7. commit your changes with a correct message, e.g. `git commit -m "[#1] add: CONTRIBUTING md"`
8. push your changes to GitLab, e.g. `git push`
9. set your Issue to "to review" if your changes are done

Now somebody else reviews your ticket and merges OR you review somebody else's issue:

1. take a issue from "to review" and push it to "in review"
2. check the issue, read the code, try it out, do whatever is needed to check the correctness of the proposed solution
    - best way is to `git checkout` the issue on your machine and verifing it there
3. if everything of the issue is correct, approve the MR in GitLab and mark it as ready (if the pipeline allows it). You don't have to rebase it.
4. push the ticket further from "in review" to "ready to merge"

The higher-level community members only can merge into main following

1. assign a "ready to merge" MR to yourself
2. review it, check the pipelines, do tests, etc; verify the integrity
3. add the changes to the [CHANGELOG](./CHANGELOG.md)
4. add the autor to [Authors](./AUTHORS.md) if not already done
5. merge it into main, close the ticket, remove unnecessary labels

## Write correct commit messages

You have to format your commit messages in a specific way. Say you're working on adding a new exercise called `foobar1.rs` on the issue #8. You could write the following commit message:

```txt
[#8] add foobar1.rs exercise
```

This ensures, the commit is linked to the issue and the use is clear in later merges or rebases.

If you're just fixing a bug, please use the `fix` type:

```txt
[#8] fix: make sure verify doesn't self-destruct
```

If you're updating documentation, use `docs`:

```txt
[#8] docs: add more information to Readme
```

If, and only if, you're absolutely sure you want to make a breaking change (please discuss this beforehand!), add an exclamation mark to the type and explain the breaking change in the message body:

```txt
[#8] fix!: completely change verification

BREAKING CHANGE: This has to be done because lorem ipsum dolor...
```

## Coding conventions

Start reading our code and you'll get the hang of it. We optimize for readability:

- We indent using two spaces (soft tabs)
- We avoid logic in views
- We ALWAYS put spaces after list items and method parameters ([1, 2, 3], not [1,2,3]), around operators (x += 1, not x+=1), and around hash arrows.
- This is public software. Consider the people who will read your code, and make it look nice for them.

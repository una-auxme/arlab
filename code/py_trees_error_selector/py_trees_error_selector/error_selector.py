#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

import itertools
import typing

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Selector


class ErrorSelector(Selector):
    def __init__(self, name: str, children=None):
        super().__init__(name, memory=True, children=children)

    def initialise(self):
        self.logger.debug(
            "%s.tick() [!RUNNING->reset current_child]" % self.__class__.__name__
        )
        self.current_child = self.children[0] if self.children else None

    def tick(self) -> typing.Iterator[Behaviour]:
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(Status.FAILURE)
            yield self
            return

        assert self.current_child is not None  # should never be true, help mypy out
        index = self.children.index(self.current_child)
        # clear out preceding status' - not actually necessary but helps
        # visualize the case of memory vs no memory
        for child in itertools.islice(self.children, None, index):
            child.stop(Status.INVALID)

        # actual work
        previous = self.current_child
        for i, child in itertools.islice(enumerate(self.children), index, None):
            for node in child.tick():
                yield node
                if node is child:
                    if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                        if node.status == Status.SUCCESS and i > 0:
                            # If we have success in an error handler:
                            # Start from first child
                            self.status = Status.RUNNING
                            self.current_child = self.children[0]
                        else:
                            self.status = node.status
                            self.current_child = child
                        if previous is None or previous != self.current_child:
                            # we interrupted, invalidate everything at a lower priority
                            passed = False
                            for child in self.children:
                                if passed:
                                    if child.status != Status.INVALID:
                                        child.stop(Status.INVALID)
                                passed = True if child == self.current_child else passed
                        yield self
                        return

        # all children failed, set failure ourselves and
        # current child to the last bugger who failed us
        self.status = Status.FAILURE
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self

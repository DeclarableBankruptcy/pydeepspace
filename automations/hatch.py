from components.hatch import Hatch
from components.alignment import Aligner
from magicbot import StateMachine, state, timed_state


class HatchController(StateMachine):

    align: Aligner
    hatch: Hatch

    def start_punch(self, force=False):
        self.engage(initial_state="wait_for_align", force=force)

    @state(first=True, must_finish=True)
    def wait_for_align(self, initial_call):
        if initial_call:
            self.align.align()
        if not self.align.is_executing:
            if self.align.successful:
                self.next_state("punching")
            else:
                self.done()

    @state(first=True, must_finish=True)
    def punching(self):
        if self.hatch.hatch_in():
            self.hatch.punch()
        if not self.hatch.hatch_in():
            self.next_state("retracting")

    @timed_state(must_finish=True, duration=2.2)
    def retracting(self, state_tm):
        if state_tm > 2:
            self.hatch.retract()
            self.done()

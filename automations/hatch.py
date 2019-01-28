from components.hatch import Hatch
from automations.alignment import Aligner
from magicbot import StateMachine, state, timed_state


class HatchController(StateMachine):
    hatch: Hatch
    align: Aligner

    def punch(self, force=False):
        self.engage(force=force)

    @state(first=True, must_finish=True)
    def aligning(self, initial_call):
        if initial_call:
            self.align.mode = self.align.hatch_deposit
        self.align.engage()
        if self.align.successful and not self.align.is_executing:
            self.next_state("punching")

    @state(must_finish=True)
    def punching(self, state_tm):
        self.hatch.punch_bottom()
        if state_tm > 0.05:
            self.hatch.punch_top()
            self.next_state("retracting")

    @timed_state(must_finish=True, duration=1.2)
    def retracting(self, state_tm):
        if state_tm > 1:
            self.hatch.retract()
            self.done()

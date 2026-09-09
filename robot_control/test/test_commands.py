"""The command table names methods that live elsewhere, so it has to be pinned.

Nothing here talks to a robot: it checks that what the table promises about
each method is still what the method does.
"""

import inspect

import pytest

from robot_control.commands import COMMANDS, parse_params, render_help
from robot_control.rws.interface import RWSInterface
from robot_control.rws.provider import RWSResult


def _signature(name: str) -> inspect.Signature:
    method = getattr(RWSInterface, name, None)
    assert method is not None, f"{name} is not a method of RWSInterface"
    return inspect.signature(method)


@pytest.mark.parametrize("name", sorted(COMMANDS))
def test_command_answers_with_an_rwsresult(name):
    """A bare bool or a tuple cannot be packed into the service response."""
    assert _signature(name).return_annotation is RWSResult


@pytest.mark.parametrize("name", sorted(COMMANDS))
def test_declared_arguments_match_the_method(name):
    parameters = [p for p in _signature(name).parameters.values() if p.name != "self"]
    required = [p.name for p in parameters if p.default is inspect.Parameter.empty]
    optional = {p.name for p in parameters if p.default is not inspect.Parameter.empty}

    assert list(COMMANDS[name].args) == required
    # An optional argument may be left out of the table, never invented.
    assert set(COMMANDS[name].optional) <= optional


def test_only_the_two_chosen_commands_may_interrupt_a_trajectory():
    """Reads are always safe; among the writes only these two were meant to be."""
    writes = {
        name
        for name, command in COMMANDS.items()
        if command.while_moving and not name.startswith("get_")
    }
    assert writes == {"stop_rapid_script", "set_speedratio"}


def test_parse_params_takes_names_in_any_order():
    kwargs = parse_params(
        "set_io_signal",
        COMMANDS["set_io_signal"],
        ["signal_value=1", "signal_name=DO_1"],
    )
    assert kwargs == {"signal_name": "DO_1", "signal_value": "1"}


def test_parse_params_keeps_an_empty_value():
    """The positional form dropped these, silently shifting everything after."""
    kwargs = parse_params(
        "set_io_signal",
        COMMANDS["set_io_signal"],
        ["signal_name=DO_1", "signal_value="],
    )
    assert kwargs["signal_value"] == ""


@pytest.mark.parametrize(
    "params",
    [
        ["DO_1"],
        ["signal_name=DO_1"],
        ["signal_name=DO_1", "signal_value=1", "colour=red"],
        ["signal_name=DO_1", "signal_name=DO_2", "signal_value=1"],
    ],
)
def test_parse_params_refuses(params):
    with pytest.raises(ValueError):
        parse_params("set_io_signal", COMMANDS["set_io_signal"], params)


def test_help_lists_every_command():
    text = render_help()
    for name in COMMANDS:
        assert name in text

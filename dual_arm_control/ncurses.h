#pragma once

/*
 * HumanoidArms_SDK headers include <ncurses.h>, but the Qt integration only
 * uses the SDK's motion/query declarations and does not rely on any curses
 * API. A tiny shim keeps local builds self-contained when the ncurses dev
 * headers are absent.
 */

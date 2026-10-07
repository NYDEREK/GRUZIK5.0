#pragma once

// =============================================================================
//  commands.h - text command interpreter
// =============================================================================
//
//  One "Key=Value" command per line, from the app (TCP) or the USB console.
//  Unknown keys are ignored.
// =============================================================================

void commands_execute(const char *line);

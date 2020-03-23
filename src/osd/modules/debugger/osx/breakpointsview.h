// license:BSD-3-Clause
// copyright-holders:Vas Crabb
//============================================================
//
//  breakpointsview.h - MacOS X Cocoa debug window handling
//
//============================================================

#import "debugosx.h"

#import "debugview.h"


#import <Cocoa/Cocoa.h>


@interface MAMEBreakpointsView : MAMEDebugView
{
}

- (id)initWithFrame:(NSRect)f machine:(running_machine &)m;

@end

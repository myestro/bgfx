$input a_position, a_color0
$output v_color0

uniform vec4 u_pointSize;

/*
 * Copyright 2011-2017 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include "../common/common.sh"

void main()
{
	gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0) );
	v_color0 = a_color0;

	// http://stackoverflow.com/questions/25780145/gl-pointsize-corresponding-to-world-space-size
	POINTSIZE = u_pointSize[0] * u_pointSize[1] / gl_Position.w;
}

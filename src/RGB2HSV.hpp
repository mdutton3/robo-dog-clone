/*
 * @author Shervin Emami
 * @url http://www.shervinemami.info/colorConversion.html
 * @note [MRD] Pulled this code from Shervin's blog in 2011-12.
 *
 */

template<typename RGB, typename HSV>
void convertRGBtoHSV(RGB const & rgb, HSV & hsv)
{
	float fR, fG, fB;
	float fH, fS, fV;
	static const float FLOAT_TO_BYTE = 255.0f;
	static const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

	// Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
	int bB = rgb.b; // Blue component
	int bG = rgb.g; // Green component
	int bR = rgb.r; // Red component

	// Convert from 8-bit integers to floats.
	fR = bR * BYTE_TO_FLOAT;
	fG = bG * BYTE_TO_FLOAT;
	fB = bB * BYTE_TO_FLOAT;

	// Convert from RGB to HSV, using float ranges 0.0 to 1.0.
	float fDelta;
	float fMin, fMax;
	int iMax;
	// Get the min and max, but use integer comparisons for slight speedup.
	if (bB < bG)
	{
		if (bB < bR)
		{
			fMin = fB;
			if (bR > bG)
			{
				iMax = bR;
				fMax = fR;
			}
			else
			{
				iMax = bG;
				fMax = fG;
			}
		}
		else
		{
			fMin = fR;
			fMax = fG;
			iMax = bG;
		}
	}
	else
	{
		if (bG < bR)
		{
			fMin = fG;
			if (bB > bR)
			{
				fMax = fB;
				iMax = bB;
			}
			else
			{
				fMax = fR;
				iMax = bR;
			}
		}
		else
		{
			fMin = fR;
			fMax = fB;
			iMax = bB;
		}
	}
	fDelta = fMax - fMin;
	fV = fMax; // Value (Brightness).
	if (iMax != 0)
	{ // Make sure its not pure black.
		fS = fDelta / fMax; // Saturation.
		float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta); // Make the Hues between 0.0 to 1.0 instead of 6.0
		if (iMax == bR)
		{ // between yellow and magenta.
			fH = (fG - fB) * ANGLE_TO_UNIT;
		}
		else if (iMax == bG)
		{ // between cyan and yellow.
			fH = (2.0f / 6.0f) + (fB - fR) * ANGLE_TO_UNIT;
		}
		else
		{ // between magenta and cyan.
			fH = (4.0f / 6.0f) + (fR - fG) * ANGLE_TO_UNIT;
		}
		// Wrap outlier Hues around the circle.
		if (fH < 0.0f)
			fH += 1.0f;
		if (fH >= 1.0f)
			fH -= 1.0f;
	}
	else
	{
		// color is pure Black.
		fS = 0;
		fH = 0; // undefined hue
	}

	// Convert from floats to 8-bit integers.
	int bH = (int) (0.5f + fH * 255.0f);
	int bS = (int) (0.5f + fS * 255.0f);
	int bV = (int) (0.5f + fV * 255.0f);

	// Clip the values to make sure it fits within the 8bits.
	if (bH > 255)
		bH = 255;
	if (bH < 0)
		bH = 0;
	if (bS > 255)
		bS = 255;
	if (bS < 0)
		bS = 0;
	if (bV > 255)
		bV = 255;
	if (bV < 0)
		bV = 0;

	// Set the HSV pixel components.
	hsv.h = bH;
	hsv.s = bS;
	hsv.v = bV;
}

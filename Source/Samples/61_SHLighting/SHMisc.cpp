//=============================================================================
// Copyright (c) 2016 Lumak Software
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
//=============================================================================

#include <Urho3D/Urho3D.h>
#include <Urho3D/Math/Vector3.h>

#include "SHMisc.h"

//=============================================================================
// true area of triangle calc
//=============================================================================
float AreaOfTriangle(const Vector3 &v0, const Vector3 &v1, const Vector3 &v2)
{
    const Vector3 e0 = v1 - v0;
    const Vector3 e1 = v2 - v1;
    float area = e0.CrossProduct( e1 ).Length() * 0.5f;
    return area;
}

//=============================================================================
// extract directory and the filenam w/o ext from a full path filename
//=============================================================================
void ExtractFileDirAndName(const String &strFullPathFilename, String &strDir, String &strFilenameNoExt, char delimiter)
{
    strDir = strFullPathFilename.Substring( 0, strFullPathFilename.FindLast( '/' ) + 1);
    String strFilenameExt = strFullPathFilename.Substring( strFullPathFilename.FindLast( '/' ) + 1 );
    Vector<String> vsplitFilenameExt = strFilenameExt.Split( '.' );

    String strNameNoExt;

    // should always be true
    if ( vsplitFilenameExt.Size() > 0 )
    {
        // the delimiter signifies it's been modded already
        if ( vsplitFilenameExt[ 0 ].Find( delimiter ) != String::NPOS  )
        {
            Vector<String> vstr = vsplitFilenameExt[0].Split( delimiter );

            if ( vstr.Size() > 0  )
            {
                strNameNoExt = vstr[ 0 ];
            }
            else
            {
                strNameNoExt = vsplitFilenameExt[ 0 ];
            }
        }
        else
        {
            strNameNoExt = vsplitFilenameExt[ 0 ];
        }
    }

    strFilenameNoExt = strNameNoExt;
}




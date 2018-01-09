#include "feature.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>

#include <emmintrin.h>

#include "pyramid.hpp"
#include "helper.hpp"
static const size_t ALIGNMENT = 16;

using namespace std;

/* Based on the FAST corner detector, contributed to OpenCV by Edward Rosten.
   Below is the original copyright and the references */
/*
Copyright (c) 2006, 2008 Edward Rosten
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  *Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

  *Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  *Neither the name of the University of Cambridge nor the names of
   its contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
The references are:
 * Machine learning for high-speed corner detection,
   E. Rosten and T. Drummond, ECCV 2006
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
*/


namespace rgbdvo {

struct Point
{
  int x;
  int y;
  Point() {}
  Point(int x_, int y_) : x(x_), y(y_) {}
};

static int fast9CornerScore(const unsigned char* p, const int pixel[], int bstart)
{
    int bmin = bstart;
    int bmax = 255;
    int b = (bmax + bmin)/2;

    /*Compute the score using binary search*/
    for(;;)
    {
        int cb = *p + b;
        int c_b= *p - b;

        if( p[pixel[0]] > cb)
         if( p[pixel[1]] > cb)
          if( p[pixel[2]] > cb)
           if( p[pixel[3]] > cb)
            if( p[pixel[4]] > cb)
             if( p[pixel[5]] > cb)
              if( p[pixel[6]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 goto is_a_corner;
                else
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
               else if( p[pixel[7]] < c_b)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[14]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else if( p[pixel[6]] < c_b)
               if( p[pixel[15]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[14]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[13]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[11]] < c_b)
                      if( p[pixel[12]] < c_b)
                       if( p[pixel[14]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       if( p[pixel[14]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[13]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[14]] < c_b)
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else if( p[pixel[5]] < c_b)
              if( p[pixel[14]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       if( p[pixel[11]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[12]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[11]] < c_b)
                      if( p[pixel[13]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[14]] < c_b)
               if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 if( p[pixel[9]] < c_b)
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[12]] < c_b)
                     if( p[pixel[13]] < c_b)
                      if( p[pixel[6]] < c_b)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               if( p[pixel[6]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      if( p[pixel[13]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       if( p[pixel[11]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[12]] < c_b)
               if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 if( p[pixel[9]] < c_b)
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[6]] < c_b)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else if( p[pixel[4]] < c_b)
             if( p[pixel[13]] > cb)
              if( p[pixel[11]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[11]] < c_b)
               if( p[pixel[5]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[7]] < c_b)
                  if( p[pixel[8]] < c_b)
                   if( p[pixel[9]] < c_b)
                    if( p[pixel[10]] < c_b)
                     if( p[pixel[12]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[13]] < c_b)
              if( p[pixel[7]] < c_b)
               if( p[pixel[8]] < c_b)
                if( p[pixel[9]] < c_b)
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[6]] < c_b)
                     if( p[pixel[5]] < c_b)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[15]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              if( p[pixel[5]] < c_b)
               if( p[pixel[6]] < c_b)
                if( p[pixel[7]] < c_b)
                 if( p[pixel[8]] < c_b)
                  if( p[pixel[9]] < c_b)
                   if( p[pixel[10]] < c_b)
                    if( p[pixel[11]] < c_b)
                     if( p[pixel[12]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      if( p[pixel[10]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[11]] < c_b)
              if( p[pixel[7]] < c_b)
               if( p[pixel[8]] < c_b)
                if( p[pixel[9]] < c_b)
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[6]] < c_b)
                     if( p[pixel[5]] < c_b)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] < c_b)
                      if( p[pixel[15]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else if( p[pixel[3]] < c_b)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[7]] < c_b)
              if( p[pixel[8]] < c_b)
               if( p[pixel[9]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[6]] < c_b)
                  if( p[pixel[5]] < c_b)
                   if( p[pixel[4]] < c_b)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] < c_b)
                     if( p[pixel[13]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     if( p[pixel[15]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     if( p[pixel[9]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[7]] < c_b)
              if( p[pixel[8]] < c_b)
               if( p[pixel[9]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[5]] < c_b)
                    if( p[pixel[4]] < c_b)
                     goto is_a_corner;
                    else
                     if( p[pixel[13]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    if( p[pixel[13]] < c_b)
                     if( p[pixel[14]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     if( p[pixel[15]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[7]] < c_b)
             if( p[pixel[8]] < c_b)
              if( p[pixel[10]] < c_b)
               if( p[pixel[6]] < c_b)
                if( p[pixel[5]] < c_b)
                 if( p[pixel[4]] < c_b)
                  if( p[pixel[3]] < c_b)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] < c_b)
                    if( p[pixel[12]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    if( p[pixel[15]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    if( p[pixel[8]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[7]] < c_b)
             if( p[pixel[8]] < c_b)
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[6]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[4]] < c_b)
                   if( p[pixel[3]] < c_b)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] < c_b)
                    if( p[pixel[13]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    if( p[pixel[14]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    if( p[pixel[15]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
         else if( p[pixel[1]] < c_b)
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[7]] < c_b)
            if( p[pixel[9]] < c_b)
             if( p[pixel[6]] < c_b)
              if( p[pixel[5]] < c_b)
               if( p[pixel[4]] < c_b)
                if( p[pixel[3]] < c_b)
                 if( p[pixel[2]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] < c_b)
                   if( p[pixel[11]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] < c_b)
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   if( p[pixel[15]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[11]] > cb)
              if( p[pixel[12]] > cb)
               if( p[pixel[13]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[15]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[7]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[7]] < c_b)
            if( p[pixel[9]] < c_b)
             if( p[pixel[10]] < c_b)
              if( p[pixel[6]] < c_b)
               if( p[pixel[5]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[3]] < c_b)
                  if( p[pixel[2]] < c_b)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] < c_b)
                   if( p[pixel[12]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   if( p[pixel[13]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   if( p[pixel[14]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   if( p[pixel[15]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
        else if( p[pixel[0]] < c_b)
         if( p[pixel[1]] > cb)
          if( p[pixel[8]] > cb)
           if( p[pixel[7]] > cb)
            if( p[pixel[9]] > cb)
             if( p[pixel[6]] > cb)
              if( p[pixel[5]] > cb)
               if( p[pixel[4]] > cb)
                if( p[pixel[3]] > cb)
                 if( p[pixel[2]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] > cb)
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   if( p[pixel[15]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else if( p[pixel[1]] < c_b)
          if( p[pixel[2]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[7]] > cb)
             if( p[pixel[8]] > cb)
              if( p[pixel[10]] > cb)
               if( p[pixel[6]] > cb)
                if( p[pixel[5]] > cb)
                 if( p[pixel[4]] > cb)
                  if( p[pixel[3]] > cb)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] > cb)
                    if( p[pixel[12]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    if( p[pixel[15]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[2]] < c_b)
           if( p[pixel[3]] > cb)
            if( p[pixel[10]] > cb)
             if( p[pixel[7]] > cb)
              if( p[pixel[8]] > cb)
               if( p[pixel[9]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[6]] > cb)
                  if( p[pixel[5]] > cb)
                   if( p[pixel[4]] > cb)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] > cb)
                     if( p[pixel[13]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     if( p[pixel[15]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[3]] < c_b)
            if( p[pixel[4]] > cb)
             if( p[pixel[13]] > cb)
              if( p[pixel[7]] > cb)
               if( p[pixel[8]] > cb)
                if( p[pixel[9]] > cb)
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    if( p[pixel[6]] > cb)
                     if( p[pixel[5]] > cb)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] > cb)
                      if( p[pixel[15]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[13]] < c_b)
              if( p[pixel[11]] > cb)
               if( p[pixel[5]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[12]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[11]] < c_b)
               if( p[pixel[12]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              if( p[pixel[5]] > cb)
               if( p[pixel[6]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else if( p[pixel[4]] < c_b)
             if( p[pixel[5]] > cb)
              if( p[pixel[14]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 if( p[pixel[9]] > cb)
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    if( p[pixel[12]] > cb)
                     if( p[pixel[13]] > cb)
                      if( p[pixel[6]] > cb)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[14]] < c_b)
               if( p[pixel[12]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[11]] > cb)
                      if( p[pixel[13]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[12]] < c_b)
                if( p[pixel[13]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       if( p[pixel[11]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               if( p[pixel[6]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else if( p[pixel[5]] < c_b)
              if( p[pixel[6]] > cb)
               if( p[pixel[15]] < c_b)
                if( p[pixel[13]] > cb)
                 if( p[pixel[7]] > cb)
                  if( p[pixel[8]] > cb)
                   if( p[pixel[9]] > cb)
                    if( p[pixel[10]] > cb)
                     if( p[pixel[11]] > cb)
                      if( p[pixel[12]] > cb)
                       if( p[pixel[14]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[13]] < c_b)
                 if( p[pixel[14]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       if( p[pixel[14]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else if( p[pixel[6]] < c_b)
               if( p[pixel[7]] > cb)
                if( p[pixel[14]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[13]] > cb)
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[7]] < c_b)
                if( p[pixel[8]] < c_b)
                 goto is_a_corner;
                else
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[13]] > cb)
                if( p[pixel[7]] > cb)
                 if( p[pixel[8]] > cb)
                  if( p[pixel[9]] > cb)
                   if( p[pixel[10]] > cb)
                    if( p[pixel[11]] > cb)
                     if( p[pixel[12]] > cb)
                      if( p[pixel[14]] > cb)
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[12]] > cb)
               if( p[pixel[7]] > cb)
                if( p[pixel[8]] > cb)
                 if( p[pixel[9]] > cb)
                  if( p[pixel[10]] > cb)
                   if( p[pixel[11]] > cb)
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      if( p[pixel[6]] > cb)
                       goto is_a_corner;
                      else
                       if( p[pixel[15]] > cb)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       if( p[pixel[11]] < c_b)
                        goto is_a_corner;
                       else
                        goto is_not_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[11]] > cb)
              if( p[pixel[7]] > cb)
               if( p[pixel[8]] > cb)
                if( p[pixel[9]] > cb)
                 if( p[pixel[10]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[6]] > cb)
                     if( p[pixel[5]] > cb)
                      goto is_a_corner;
                     else
                      if( p[pixel[14]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                    else
                     if( p[pixel[14]] > cb)
                      if( p[pixel[15]] > cb)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      if( p[pixel[10]] < c_b)
                       goto is_a_corner;
                      else
                       goto is_not_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            if( p[pixel[10]] > cb)
             if( p[pixel[7]] > cb)
              if( p[pixel[8]] > cb)
               if( p[pixel[9]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[6]] > cb)
                   if( p[pixel[5]] > cb)
                    if( p[pixel[4]] > cb)
                     goto is_a_corner;
                    else
                     if( p[pixel[13]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                   else
                    if( p[pixel[13]] > cb)
                     if( p[pixel[14]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     if( p[pixel[15]] > cb)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     if( p[pixel[9]] < c_b)
                      goto is_a_corner;
                     else
                      goto is_not_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
          else
           if( p[pixel[9]] > cb)
            if( p[pixel[7]] > cb)
             if( p[pixel[8]] > cb)
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[6]] > cb)
                 if( p[pixel[5]] > cb)
                  if( p[pixel[4]] > cb)
                   if( p[pixel[3]] > cb)
                    goto is_a_corner;
                   else
                    if( p[pixel[12]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                  else
                   if( p[pixel[12]] > cb)
                    if( p[pixel[13]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    if( p[pixel[14]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    if( p[pixel[15]] > cb)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    if( p[pixel[8]] < c_b)
                     goto is_a_corner;
                    else
                     goto is_not_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
         else
          if( p[pixel[8]] > cb)
           if( p[pixel[7]] > cb)
            if( p[pixel[9]] > cb)
             if( p[pixel[10]] > cb)
              if( p[pixel[6]] > cb)
               if( p[pixel[5]] > cb)
                if( p[pixel[4]] > cb)
                 if( p[pixel[3]] > cb)
                  if( p[pixel[2]] > cb)
                   goto is_a_corner;
                  else
                   if( p[pixel[11]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                 else
                  if( p[pixel[11]] > cb)
                   if( p[pixel[12]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   if( p[pixel[13]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   if( p[pixel[14]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   if( p[pixel[15]] > cb)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[10]] < c_b)
             if( p[pixel[11]] < c_b)
              if( p[pixel[12]] < c_b)
               if( p[pixel[13]] < c_b)
                if( p[pixel[14]] < c_b)
                 if( p[pixel[15]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[2]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[4]] < c_b)
                 if( p[pixel[5]] < c_b)
                  if( p[pixel[6]] < c_b)
                   if( p[pixel[7]] < c_b)
                    goto is_a_corner;
                   else
                    goto is_not_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
        else
         if( p[pixel[7]] > cb)
          if( p[pixel[8]] > cb)
           if( p[pixel[9]] > cb)
            if( p[pixel[6]] > cb)
             if( p[pixel[5]] > cb)
              if( p[pixel[4]] > cb)
               if( p[pixel[3]] > cb)
                if( p[pixel[2]] > cb)
                 if( p[pixel[1]] > cb)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] > cb)
                  if( p[pixel[11]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] > cb)
                 if( p[pixel[11]] > cb)
                  if( p[pixel[12]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] > cb)
                if( p[pixel[11]] > cb)
                 if( p[pixel[12]] > cb)
                  if( p[pixel[13]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] > cb)
               if( p[pixel[11]] > cb)
                if( p[pixel[12]] > cb)
                 if( p[pixel[13]] > cb)
                  if( p[pixel[14]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[10]] > cb)
              if( p[pixel[11]] > cb)
               if( p[pixel[12]] > cb)
                if( p[pixel[13]] > cb)
                 if( p[pixel[14]] > cb)
                  if( p[pixel[15]] > cb)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else if( p[pixel[7]] < c_b)
          if( p[pixel[8]] < c_b)
           if( p[pixel[9]] < c_b)
            if( p[pixel[6]] < c_b)
             if( p[pixel[5]] < c_b)
              if( p[pixel[4]] < c_b)
               if( p[pixel[3]] < c_b)
                if( p[pixel[2]] < c_b)
                 if( p[pixel[1]] < c_b)
                  goto is_a_corner;
                 else
                  if( p[pixel[10]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                else
                 if( p[pixel[10]] < c_b)
                  if( p[pixel[11]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
               else
                if( p[pixel[10]] < c_b)
                 if( p[pixel[11]] < c_b)
                  if( p[pixel[12]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
              else
               if( p[pixel[10]] < c_b)
                if( p[pixel[11]] < c_b)
                 if( p[pixel[12]] < c_b)
                  if( p[pixel[13]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
             else
              if( p[pixel[10]] < c_b)
               if( p[pixel[11]] < c_b)
                if( p[pixel[12]] < c_b)
                 if( p[pixel[13]] < c_b)
                  if( p[pixel[14]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
            else
             if( p[pixel[10]] < c_b)
              if( p[pixel[11]] < c_b)
               if( p[pixel[12]] < c_b)
                if( p[pixel[13]] < c_b)
                 if( p[pixel[14]] < c_b)
                  if( p[pixel[15]] < c_b)
                   goto is_a_corner;
                  else
                   goto is_not_a_corner;
                 else
                  goto is_not_a_corner;
                else
                 goto is_not_a_corner;
               else
                goto is_not_a_corner;
              else
               goto is_not_a_corner;
             else
              goto is_not_a_corner;
           else
            goto is_not_a_corner;
          else
           goto is_not_a_corner;
         else
          goto is_not_a_corner;

        is_a_corner:
            bmin=b;
            goto end_if;

        is_not_a_corner:
            bmax=b;
            goto end_if;

        end_if:

        if(bmin == bmax - 1 || bmin == bmax)
            return bmin;
        b = (bmin + bmax) / 2;
    }
}

static inline void makeOffsets(int pixel[], int row_stride)
{
    pixel[0] = 0 + row_stride * 3;
    pixel[1] = 1 + row_stride * 3;
    pixel[2] = 2 + row_stride * 2;
    pixel[3] = 3 + row_stride * 1;
    pixel[4] = 3 + row_stride * 0;
    pixel[5] = 3 + row_stride * -1;
    pixel[6] = 2 + row_stride * -2;
    pixel[7] = 1 + row_stride * -3;
    pixel[8] = 0 + row_stride * -3;
    pixel[9] = -1 + row_stride * -3;
    pixel[10] = -2 + row_stride * -2;
    pixel[11] = -3 + row_stride * -1;
    pixel[12] = -3 + row_stride * 0;
    pixel[13] = -3 + row_stride * 1;
    pixel[14] = -2 + row_stride * 2;
    pixel[15] = -1 + row_stride * 3;
}


static void fast9ComputeScores(const uint8_t* img, int width, int height, int row_stride,
        vector<Point>& corners, vector<int>& scores, int b)
{
    int pixel[16];
    makeOffsets(pixel, (int)row_stride);
    size_t n, num_corners = corners.size();

    scores.resize(num_corners);
    for(n=0; n < num_corners; n++) {
        const uint8_t* p = &img[corners[n].y*row_stride + corners[n].x];
        scores[n] = fast9CornerScore(p, pixel, b);
    }
}


static void fast9Detect(const uint8_t* img, int width, int height, int row_stride,
        vector<Point>& ret_corners, int b)
{
    int xsize = width, ysize = height;
    int pixel[16];
    makeOffsets(pixel, row_stride);

    ret_corners.reserve(512);

    for(int y=3; y < ysize - 3; y++)
        for(int x=3; x < xsize - 3; x++)
        {
//            const uchar* p = &img.at<uchar>(y,x);
            const uint8_t* p = &img[y*row_stride + x];

            int cb = *p + b;
            int c_b= *p - b;
        if(p[pixel[0]] > cb)
         if(p[pixel[1]] > cb)
          if(p[pixel[2]] > cb)
           if(p[pixel[3]] > cb)
            if(p[pixel[4]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 {}
                else
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[15]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  {}
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[12]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       if(p[pixel[14]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[14]] < c_b)
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[14]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[11]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      if(p[pixel[13]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       if(p[pixel[11]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 if(p[pixel[9]] < c_b)
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[6]] < c_b)
                       {}
                      else
                       if(p[pixel[15]] < c_b)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[13]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[7]] < c_b)
                  if(p[pixel[8]] < c_b)
                   if(p[pixel[9]] < c_b)
                    if(p[pixel[10]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[7]] < c_b)
                 if(p[pixel[8]] < c_b)
                  if(p[pixel[9]] < c_b)
                   if(p[pixel[10]] < c_b)
                    if(p[pixel[11]] < c_b)
                     if(p[pixel[12]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      if(p[pixel[10]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[7]] < c_b)
               if(p[pixel[8]] < c_b)
                if(p[pixel[9]] < c_b)
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[6]] < c_b)
                     if(p[pixel[5]] < c_b)
                      {}
                     else
                      if(p[pixel[14]] < c_b)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] < c_b)
                      if(p[pixel[15]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[6]] < c_b)
                  if(p[pixel[5]] < c_b)
                   if(p[pixel[4]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     if(p[pixel[9]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[7]] < c_b)
              if(p[pixel[8]] < c_b)
               if(p[pixel[9]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[5]] < c_b)
                    if(p[pixel[4]] < c_b)
                     {}
                    else
                     if(p[pixel[13]] < c_b)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] < c_b)
                     if(p[pixel[14]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     if(p[pixel[15]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[6]] < c_b)
                if(p[pixel[5]] < c_b)
                 if(p[pixel[4]] < c_b)
                  if(p[pixel[3]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    if(p[pixel[8]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[7]] < c_b)
             if(p[pixel[8]] < c_b)
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[6]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[4]] < c_b)
                   if(p[pixel[3]] < c_b)
                    {}
                   else
                    if(p[pixel[12]] < c_b)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] < c_b)
                    if(p[pixel[13]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    if(p[pixel[14]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    if(p[pixel[15]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[6]] < c_b)
              if(p[pixel[5]] < c_b)
               if(p[pixel[4]] < c_b)
                if(p[pixel[3]] < c_b)
                 if(p[pixel[2]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[11]] > cb)
              if(p[pixel[12]] > cb)
               if(p[pixel[13]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[15]] > cb)
                  {}
                 else
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[7]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[7]] < c_b)
            if(p[pixel[9]] < c_b)
             if(p[pixel[10]] < c_b)
              if(p[pixel[6]] < c_b)
               if(p[pixel[5]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[3]] < c_b)
                  if(p[pixel[2]] < c_b)
                   {}
                  else
                   if(p[pixel[11]] < c_b)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] < c_b)
                   if(p[pixel[12]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   if(p[pixel[13]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   if(p[pixel[14]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   if(p[pixel[15]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else if(p[pixel[0]] < c_b)
         if(p[pixel[1]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[6]] > cb)
              if(p[pixel[5]] > cb)
               if(p[pixel[4]] > cb)
                if(p[pixel[3]] > cb)
                 if(p[pixel[2]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[1]] < c_b)
          if(p[pixel[2]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[5]] > cb)
                 if(p[pixel[4]] > cb)
                  if(p[pixel[3]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[2]] < c_b)
           if(p[pixel[3]] > cb)
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[6]] > cb)
                  if(p[pixel[5]] > cb)
                   if(p[pixel[4]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[3]] < c_b)
            if(p[pixel[4]] > cb)
             if(p[pixel[13]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[13]] < c_b)
              if(p[pixel[11]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              if(p[pixel[5]] > cb)
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else if(p[pixel[4]] < c_b)
             if(p[pixel[5]] > cb)
              if(p[pixel[14]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[12]] > cb)
                     if(p[pixel[13]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[14]] < c_b)
               if(p[pixel[12]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               if(p[pixel[6]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(p[pixel[5]] < c_b)
              if(p[pixel[6]] > cb)
               if(p[pixel[15]] < c_b)
                if(p[pixel[13]] > cb)
                 if(p[pixel[7]] > cb)
                  if(p[pixel[8]] > cb)
                   if(p[pixel[9]] > cb)
                    if(p[pixel[10]] > cb)
                     if(p[pixel[11]] > cb)
                      if(p[pixel[12]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[14]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else if(p[pixel[6]] < c_b)
               if(p[pixel[7]] > cb)
                if(p[pixel[14]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[13]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[7]] < c_b)
                if(p[pixel[8]] < c_b)
                 {}
                else
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
               else
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[13]] > cb)
                if(p[pixel[7]] > cb)
                 if(p[pixel[8]] > cb)
                  if(p[pixel[9]] > cb)
                   if(p[pixel[10]] > cb)
                    if(p[pixel[11]] > cb)
                     if(p[pixel[12]] > cb)
                      if(p[pixel[14]] > cb)
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[12]] > cb)
               if(p[pixel[7]] > cb)
                if(p[pixel[8]] > cb)
                 if(p[pixel[9]] > cb)
                  if(p[pixel[10]] > cb)
                   if(p[pixel[11]] > cb)
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      if(p[pixel[6]] > cb)
                       {}
                      else
                       if(p[pixel[15]] > cb)
                        {}
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       if(p[pixel[11]] < c_b)
                        {}
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[11]] > cb)
              if(p[pixel[7]] > cb)
               if(p[pixel[8]] > cb)
                if(p[pixel[9]] > cb)
                 if(p[pixel[10]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[6]] > cb)
                     if(p[pixel[5]] > cb)
                      {}
                     else
                      if(p[pixel[14]] > cb)
                       {}
                      else
                       continue;
                    else
                     if(p[pixel[14]] > cb)
                      if(p[pixel[15]] > cb)
                       {}
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      if(p[pixel[10]] < c_b)
                       {}
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            if(p[pixel[10]] > cb)
             if(p[pixel[7]] > cb)
              if(p[pixel[8]] > cb)
               if(p[pixel[9]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[6]] > cb)
                   if(p[pixel[5]] > cb)
                    if(p[pixel[4]] > cb)
                     {}
                    else
                     if(p[pixel[13]] > cb)
                      {}
                     else
                      continue;
                   else
                    if(p[pixel[13]] > cb)
                     if(p[pixel[14]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                  else
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     if(p[pixel[15]] > cb)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     if(p[pixel[9]] < c_b)
                      {}
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;
            else
             continue;
          else
           if(p[pixel[9]] > cb)
            if(p[pixel[7]] > cb)
             if(p[pixel[8]] > cb)
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[6]] > cb)
                 if(p[pixel[5]] > cb)
                  if(p[pixel[4]] > cb)
                   if(p[pixel[3]] > cb)
                    {}
                   else
                    if(p[pixel[12]] > cb)
                     {}
                    else
                     continue;
                  else
                   if(p[pixel[12]] > cb)
                    if(p[pixel[13]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                 else
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    if(p[pixel[14]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    if(p[pixel[15]] > cb)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else
               continue;
             else
              continue;
            else
             continue;
           else if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    if(p[pixel[8]] < c_b)
                     {}
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
         else
          if(p[pixel[8]] > cb)
           if(p[pixel[7]] > cb)
            if(p[pixel[9]] > cb)
             if(p[pixel[10]] > cb)
              if(p[pixel[6]] > cb)
               if(p[pixel[5]] > cb)
                if(p[pixel[4]] > cb)
                 if(p[pixel[3]] > cb)
                  if(p[pixel[2]] > cb)
                   {}
                  else
                   if(p[pixel[11]] > cb)
                    {}
                   else
                    continue;
                 else
                  if(p[pixel[11]] > cb)
                   if(p[pixel[12]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   if(p[pixel[13]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   if(p[pixel[14]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   if(p[pixel[15]] > cb)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              continue;
            else
             continue;
           else
            continue;
          else if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[10]] < c_b)
             if(p[pixel[11]] < c_b)
              if(p[pixel[12]] < c_b)
               if(p[pixel[13]] < c_b)
                if(p[pixel[14]] < c_b)
                 if(p[pixel[15]] < c_b)
                  {}
                 else
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                else
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[2]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[4]] < c_b)
                 if(p[pixel[5]] < c_b)
                  if(p[pixel[6]] < c_b)
                   if(p[pixel[7]] < c_b)
                    {}
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             continue;
           else
            continue;
          else
           continue;
        else
         if(p[pixel[7]] > cb)
          if(p[pixel[8]] > cb)
           if(p[pixel[9]] > cb)
            if(p[pixel[6]] > cb)
             if(p[pixel[5]] > cb)
              if(p[pixel[4]] > cb)
               if(p[pixel[3]] > cb)
                if(p[pixel[2]] > cb)
                 if(p[pixel[1]] > cb)
                  {}
                 else
                  if(p[pixel[10]] > cb)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] > cb)
                  if(p[pixel[11]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] > cb)
                 if(p[pixel[11]] > cb)
                  if(p[pixel[12]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] > cb)
                if(p[pixel[11]] > cb)
                 if(p[pixel[12]] > cb)
                  if(p[pixel[13]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] > cb)
               if(p[pixel[11]] > cb)
                if(p[pixel[12]] > cb)
                 if(p[pixel[13]] > cb)
                  if(p[pixel[14]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] > cb)
              if(p[pixel[11]] > cb)
               if(p[pixel[12]] > cb)
                if(p[pixel[13]] > cb)
                 if(p[pixel[14]] > cb)
                  if(p[pixel[15]] > cb)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else if(p[pixel[7]] < c_b)
          if(p[pixel[8]] < c_b)
           if(p[pixel[9]] < c_b)
            if(p[pixel[6]] < c_b)
             if(p[pixel[5]] < c_b)
              if(p[pixel[4]] < c_b)
               if(p[pixel[3]] < c_b)
                if(p[pixel[2]] < c_b)
                 if(p[pixel[1]] < c_b)
                  {}
                 else
                  if(p[pixel[10]] < c_b)
                   {}
                  else
                   continue;
                else
                 if(p[pixel[10]] < c_b)
                  if(p[pixel[11]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
               else
                if(p[pixel[10]] < c_b)
                 if(p[pixel[11]] < c_b)
                  if(p[pixel[12]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(p[pixel[10]] < c_b)
                if(p[pixel[11]] < c_b)
                 if(p[pixel[12]] < c_b)
                  if(p[pixel[13]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(p[pixel[10]] < c_b)
               if(p[pixel[11]] < c_b)
                if(p[pixel[12]] < c_b)
                 if(p[pixel[13]] < c_b)
                  if(p[pixel[14]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
            else
             if(p[pixel[10]] < c_b)
              if(p[pixel[11]] < c_b)
               if(p[pixel[12]] < c_b)
                if(p[pixel[13]] < c_b)
                 if(p[pixel[14]] < c_b)
                  if(p[pixel[15]] < c_b)
                   {}
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               continue;
             else
              continue;
           else
            continue;
          else
           continue;
         else
          continue;
      ret_corners.push_back(Point(x,y));
        }
}

/* This defines non-strict maxima */
#define Compare(X, Y) ((X)>=(Y))

/* This is a fast, integer only, sparse nonmaximal suppression. */
/* probably only useful for FAST corner detection */
static void fastNonmaxSuppression(const vector<Point>& corners,
                                  const vector<int>& scores,
                                  vector<KeyPoint>& ret_nonmax)
{
    int i, j, num_corners = (int)corners.size();

    // Point above points (roughly) to the pixel above the one of interest, if there is a feature there.
    int point_above = 0;
    int point_below = 0;

    ret_nonmax.clear();
    if(num_corners < 1)
        return;

    /* Find where each row begins
       (the corners are output in raster scan order). A beginning of -1 signifies
       that there are no corners on that row. */
    int last_row = corners[num_corners-1].y;
    vector<int> row_start(last_row+1);

    for(i=0; i < last_row+1; i++)
        row_start[i] = -1;

    int prev_row = -1;
    for(i=0; i< num_corners; i++)
        if(corners[i].y != prev_row)
        {
            row_start[corners[i].y] = i;
            prev_row = corners[i].y;
        }

    ret_nonmax.reserve(num_corners);
    for(i = 0; i < num_corners; i++)
    {
        int score = scores[i];
        Point pos = corners[i];

        // Check left
        if(i > 0 && corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && Compare(scores[i-1], score))
            continue;

        // Check right
        if(i < num_corners-1 && corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && Compare(scores[i+1], score))
            continue;

        bool suppressed = false;
        // Check above (if there is a valid row above)
        if(pos.y != 0 && row_start[pos.y - 1] != -1)
        {
            // Make sure that current point_above is one row above.
            if(corners[point_above].y < pos.y - 1)
                point_above = row_start[pos.y-1];

            // Make point_above point to the first of the pixels above the current point, if it exists.
            for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
                ;

            for(j=point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
            {
                int x = corners[j].x;
                if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j], score))
                {
                    suppressed = true;
                    break;
                }
            }
            if( suppressed )
                continue;
        }

        // Check below (if there is anything below)
        if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < num_corners) // Nothing below
        {
            if(corners[point_below].y < pos.y + 1)
                point_below = row_start[pos.y+1];

            // Make point below point to one of the pixels belowthe current point, if it exists.
            for(; point_below < num_corners && corners[point_below].y == pos.y+1
                && corners[point_below].x < pos.x - 1; point_below++)
                ;

            for(j=point_below; j < num_corners && corners[j].y == pos.y+1 && corners[j].x <= pos.x + 1; j++)
            {
                int x = corners[j].x;
                if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && Compare(scores[j],score))
                {
                    suppressed = true;
                    break;
                }
            }
            if( suppressed )
                continue;
        }

        ret_nonmax.push_back(KeyPoint(corners[i].x, corners[i].y, (float)score));
    }
}

void FAST(const uint8_t* image, int32_t width,
          int32_t height, int32_t row_stride,
          vector<KeyPoint>* keypoints, int32_t threshold,
          bool nonmax_suppression) {

  vector<Point> corners;
  fast9Detect(image, width, height, row_stride, corners, threshold);
  if (nonmax_suppression) {
    vector<int32_t> scores;
    fast9ComputeScores(
      image, width, height, row_stride, corners, scores, threshold);
    fastNonmaxSuppression(corners, scores, *keypoints);
  } else {
    size_t i, n = corners.size();
    keypoints->resize(n);
    for(i = 0; i < n; i++) {
      (*keypoints)[i] = KeyPoint(corners[i].x, corners[i].y, 0);
    }
  }
}


void
Descriptor::Init() {
  descriptor_len_ = feature_window_size_ * feature_window_size_;

  if((descriptor_len_ - 1) % 16 == 0) {
    descriptor_len_ -= 1;
  }

  // descriptor properties
  descriptor_stride_ = round_up_to_multiple(descriptor_len_, ALIGNMENT);
  brightess_offset_num_sse_ops_ = descriptor_stride_ / 16;
  num_descriptor_pad_bytes_ = descriptor_stride_ - descriptor_len_;

  // populate the descriptor offset vector.  This vector stores the index
  // offsets of descriptor pixels for a descriptor centered around a given
  // pixel.
  descriptor_index_offsets_.reset(new int[descriptor_len_]);
  int32_t i = 0;
  int32_t dr = (feature_window_size_ - 1) / 2;
  for(int32_t y_offset = -dr; y_offset <= dr; y_offset++) {
    for(int32_t x_offset = -dr; x_offset <= dr && i<descriptor_len_;
        x_offset++) {
      descriptor_index_offsets_[i] = y_offset * raw_gray_stride_ + x_offset;
      i++;
    }
  }
  assert(i == descriptor_len_);
  assert(0 == posix_memalign((void**)&descriptor_brightness_offset_,
                             ALIGNMENT,
                             16));
}

void
Descriptor::ExtractInterpDescriptor(uint8_t *raw_gray,
                                    float x, float y,
                                    uint8_t* descriptor) const {
  int32_t u = static_cast<int32_t>(x);
  int32_t v = static_cast<int32_t>(y);
  float wright  = (x - u);
  float wbottom = (y - v);
  float wa = (1. - wright) * (1. - wbottom);
  float wb = wright        * (1. - wbottom);
  float wc = (1. - wright) * wbottom;
  float wd = wright        * wbottom;
  int32_t initial_index = v * raw_gray_stride_ + u;
  for(int32_t i = 0; i < descriptor_len_; i++) {
    int32_t k = initial_index + descriptor_index_offsets_[i];
    descriptor[i] =
        (uint8_t)(wa * raw_gray[k] +
        wb * raw_gray[k + 1] +
        wc * raw_gray[k + raw_gray_stride_] +
        wd * raw_gray[k + raw_gray_stride_ + 1]);
  }
  Normalized(descriptor);

  // zero out some pad bytes of the descriptor so that SIMD
  // operations can be simpler
  memset(descriptor + descriptor_len_, 0, num_descriptor_pad_bytes_);
}

void
Descriptor::ExtractAlignedDescriptor(uint8_t *raw_gray,
                                     int32_t x, int32_t y,
                                     uint8_t* descriptor) const {
  // require that the descriptor be 16-byte aligned
  assert(IS_ALIGNED16(descriptor));

  int32_t start = y * raw_gray_stride_ + x;
  for (int32_t i = 0; i < descriptor_len_; i++) {
    descriptor[i] = raw_gray[start + descriptor_index_offsets_[i]];
  }
  Normalized(descriptor);

  // zero out some pad bytes of the descriptor so that SIMD
  // operations can be simpler
  memset(descriptor + descriptor_len_, 0, num_descriptor_pad_bytes_);
}

void
Descriptor::Normalized(uint8_t* desc) const {
  assert(IS_ALIGNED16(descriptor_brightness_offset_));

  // get mean of patch
  uint32_t desc_mean = std::accumulate(desc, desc + descriptor_len_, 0)/descriptor_len_;
  // subtract mean, adding offset so 0 -> 128
  if(desc_mean < 128) {
    std::fill(descriptor_brightness_offset_, descriptor_brightness_offset_+16, 128-desc_mean);
    for(int op=0; op<brightess_offset_num_sse_ops_; op++) {
      ((__m128i*)desc)[op] = _mm_adds_epu8(((__m128i*)desc)[op],
        *(__m128i*)descriptor_brightness_offset_);
    }
  } else if (desc_mean > 128){
    std::fill(descriptor_brightness_offset_, descriptor_brightness_offset_+16, desc_mean-128);
    for(int op=0; op<brightess_offset_num_sse_ops_; op++) {
      ((__m128i*)desc)[op] = _mm_subs_epu8(((__m128i*)desc)[op],
        *(__m128i*)descriptor_brightness_offset_);
    }
  }
}

void
Descriptor::ExtractInterpDescriptors(uint8_t* image,
                                     const KeypointData* keypoints,
                                     int32_t num_keypoints,
                                     uint8_t* descriptors) const {
  for (int32_t n = 0; n < num_keypoints; ++n) {
    const KeypointData& kp_data(keypoints[n]);
    uint8_t* descriptor = descriptors + n * descriptor_stride_;
    ExtractInterpDescriptor(image, kp_data.kp.u, kp_data.kp.v, descriptor);
    Normalized(descriptor);
  }
}

void
Descriptor::ExtractAlignedDescriptors(uint8_t* image,
                                      const KeypointData* keypoints,
                                      int32_t num_keypoints,
                                      uint8_t* descriptors) const {
  for (int32_t n = 0; n < num_keypoints; ++n) {
    const KeypointData& kp_data(keypoints[n]);
    uint8_t* descriptor = descriptors + n * descriptor_stride_;
    ExtractAlignedDescriptor(image,
                             static_cast<int32_t>(kp_data.kp.u),
                             static_cast<int32_t>(kp_data.kp.v),
                             descriptor);
    Normalized(descriptor);
  }
}

FeatureMatcher::FeatureMatcher() :
  _ref_feature_capacity(500),
  _target_feature_capacity(500)
{
  _ref_to_target_indices = (int32_t*) malloc(_ref_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _ref_to_target_scores = (int32_t*) malloc(_ref_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _target_to_ref_indices = (int32_t*) malloc(_target_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
  _target_to_ref_scores = (int32_t*) malloc(_target_feature_capacity * sizeof(int32_t));
  if (_ref_to_target_indices == NULL) {
    perror("malloc");
  }
}

FeatureMatcher::~FeatureMatcher()
{
  free(_ref_to_target_indices);
  free(_ref_to_target_scores);
  free(_target_to_ref_indices);
  free(_target_to_ref_scores);
}

void
FeatureMatcher::matchFeatures(PyramidLevel* ref_level,
                              PyramidLevel* target_level,
                              const std::vector<std::vector<int> >& candidates,
                              FeatureMatch* matches,
                              int* num_matches)
{
  int num_ref_features = ref_level->GetNumKeypoints();
  int num_target_features = target_level->GetNumKeypoints();

  // allocate more space for buffers if necessary
  if (num_ref_features > _ref_feature_capacity) {
    _ref_to_target_indices = (int32_t*) realloc(_ref_to_target_indices, num_ref_features * sizeof(int32_t));
    _ref_to_target_scores = (int32_t*) realloc(_ref_to_target_scores, num_ref_features * sizeof(int32_t));
    _ref_feature_capacity = num_ref_features;
  }

  if (num_target_features > _target_feature_capacity) {
    int buf_size = num_target_features * sizeof(int32_t);
    _target_to_ref_indices = (int32_t*) realloc(_target_to_ref_indices, buf_size);
    _target_to_ref_scores = (int32_t*) realloc(_target_to_ref_scores, buf_size);
    _target_feature_capacity = num_target_features;
  }

  int descriptor_len = ref_level->GetDescriptorLength();
  assert(descriptor_len == target_level->GetDescriptorLength());

  SAD sad(descriptor_len);

  int32_t worst_score = sad.getWorstScore();

  // initialize book-keeping for feature matching
  for (int i = 0; i < num_ref_features; i++) {
    _ref_to_target_scores[i] = worst_score + 1;
    _ref_to_target_indices[i] = -1;
  }
  for (int i = 0; i < num_target_features; i++) {
    _target_to_ref_scores[i] = worst_score + 1;
    _target_to_ref_indices[i] = -1;
  }

  // for each feature in the target frame, compute the best matching feature in
  // the reference frame.
  // Similarly, compute the best matching feature in the target frame for each
  // feature in the reference frame.
  // Match score is defined as the sum of absolute differences between two feature
  // descriptors.  Lower scores (less difference) are better.
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    const uint8_t * ref_desc = ref_level->GetDescriptor(ref_ind);

    const std::vector<int>& ref_candidates(candidates[ref_ind]);
    for (std::vector<int>::const_iterator ref_candidates_itr = ref_candidates.begin(),
         ref_candidates_end = ref_candidates.end();
         ref_candidates_itr != ref_candidates_end;
         ++ref_candidates_itr) {
      int target_ind = *ref_candidates_itr;
      const uint8_t * target_desc = target_level->GetDescriptor(target_ind);

      int score = sad.score(ref_desc, target_desc);
      assert(score <= worst_score);

      // see if this score is the best for either descriptor
      if (score < _ref_to_target_scores[ref_ind]) {
        _ref_to_target_scores[ref_ind] = score;
        _ref_to_target_indices[ref_ind] = target_ind;
      }
      if (score < _target_to_ref_scores[target_ind]) {
        _target_to_ref_scores[target_ind] = score;
        _target_to_ref_indices[target_ind] = ref_ind;
      }
    }
  }

  // now find features that are mutual best matches in both directions
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    int target_ind = _ref_to_target_indices[ref_ind];
    if (target_ind >= 0 &&
        _target_to_ref_indices[target_ind] == ref_ind) {

      KeypointData* ref_kpdata = ref_level->GetKeypointData(ref_ind);
      KeypointData* target_kpdata = target_level->GetKeypointData(target_ind);

      assert(ref_kpdata->kp.u >= 0 && ref_kpdata->kp.v >= 0 &&
             ref_kpdata->kp.u < ref_level->GetWidth() &&
             ref_kpdata->kp.v < ref_level->GetHeight());
      assert(target_kpdata->kp.u >= 0 && target_kpdata->kp.v >= 0 &&
             target_kpdata->kp.u < target_level->GetWidth() &&
             target_kpdata->kp.v < target_level->GetHeight());

      FeatureMatch match(target_kpdata, ref_kpdata);
      match.status = MATCH_OK;
      matches[*num_matches] = match;
      (*num_matches)++;
    }
  }

}

}

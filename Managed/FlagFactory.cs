// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             ___________              __                 __________                                      __                         
//                             \__    ___/___   _______/  |_  ___________  \______   \_______   ____   ______ ____   _____/  |_                       
//                               |    |_/ __ \ /  ___/\   __\/ __ \_  __ \  |     ___/\_  __ \_/ __ \ /  ___// __ \ /    \   __\                      
//                               |    |\  ___/ \___ \  |  | \  ___/|  | \/  |    |     |  | \/\  ___/ \___ \\  ___/|   |  \  |                        
//                               |____| \___  >____  > |__|  \___  >__|     |____|     |__|    \___  >____  >\___  >___|  /__|                        
//                                          \/     \/            \/                                \/     \/     \/     \/                            
//                               _________                    .__       .__  .__          __                                                          
//                              /   _____/_____   ____   ____ |__|____  |  | |__| _______/  |_                                                        
//                              \_____  \\____ \_/ __ \_/ ___\|  \__  \ |  | |  |/  ___/\   __\                                                       
//                              /        \  |_> >  ___/\  \___|  |/ __ \|  |_|  |\___ \  |  |                                                         
//                             /_______  /   __/ \___  >\___  >__(____  /____/__/____  > |__|                                                         
//                                     \/|__|        \/     \/        \/             \/                                                               
//                                _____          __                         __  .__                                                                   
//                               /  _  \  __ ___/  |_  ____   _____   _____/  |_|__|__  __ ____                                                       
//                              /  /_\  \|  |  \   __\/  _ \ /     \ /  _ \   __\  \  \/ // __ \                                                      
//                             /    |    \  |  /|  | (  <_> )  Y Y  (  <_> )  | |  |\   /\  ___/                                                      
//                             \____|__  /____/ |__|  \____/|__|_|  /\____/|__| |__| \_/  \___  >                                                     
//                                     \/                         \/                          \/                                                      
//                               _________      .__          __  .__                                                                                  
//                              /   _____/ ____ |  |  __ ___/  |_|__| ____   ____   ______                                                            
//                              \_____  \ /  _ \|  | |  |  \   __\  |/  _ \ /    \ /  ___/                                                            
//                              /        (  <_> )  |_|  |  /|  | |  (  <_> )   |  \\___ \                                                              
//                             /_______  /\____/|____/____/ |__| |__|\____/|___|  /____  >                                                           
//                                     \/                                       \/     \/                                                             
//
//
//
//                              Tester Present Specialist Automotive Solutions Open Source License (TPSAS-OSL)
//                              Version 1.0, 2025
//
//                              Copyright (c) 2025 Tester Present Specialist Automotive Solutions
//
//
//             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//             documentation files (the "Software"), to use, modify, and distribute the Software subject to the following conditions:
//
//          1. Attribution.
//            - The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//            - Credit must be given to "Tester Present Specialist Automotive Solutions" in any documentation, public notices, or product materials 
//              that acknowledge the use of the Software.//
//
//          2. Open Source Use Only.
//            - The Software, including any modifications or derivative works (collectively, “Derivatives”), must be distributed only as open source software.
//            - Derivatives or distributions of the Software must be licensed under this license or another license approved by the Open Source Initiative 
//              (OSI) that maintains equivalent open source freedoms.
//            - The Software and any Derivatives may not be used as a component in any closed source or proprietary software product.
//
//          3. Redistribution.
//            - Redistributions of the Software, in source or binary form, must reproduce the above attribution, this license, and the disclaimer below 
//              in the documentation and/or other materials provided with the distribution.
//            - Any modifications made to the Software must be clearly marked as such, and the modified files must carry prominent notices stating that
//              changes have been made.
//
//          4. No Warranty.
//            - THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//              FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.
//            - IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
//              TORT, OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//          5. Termination.
//            - This license and the rights granted hereunder will terminate automatically upon any breach of the above conditions.
//            - Upon termination, you must cease all use and distribution of the Software and any Derivatives.
//
//          6. Entire Agreement.
//            - This license constitutes the entire agreement between you and Tester Present Specialist Automotive Solutions regarding the Software 
//              and supersedes any prior understandings or agreements, whether written or oral, regarding the subject matter herein.
//
//              By using, modifying, or distributing the Software, you agree to abide by all the terms and conditions of this license.
//
//
//                         
// ---------------------------------------------------------------------------------------------
//  FlagFactory.cs — draws the Australian National Flag programmatically with GDI+ (no image asset).
//  Union Jack canton + Commonwealth Star + Southern Cross, at the correct 2:1 proportions.
// ---------------------------------------------------------------------------------------------

using System;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace TesterPresent.Adem.EmuConsole
{
    internal static class FlagFactory
    {
        private static readonly Color Blue = Color.FromArgb(0, 33, 105);   // #002169
        private static readonly Color Red = Color.FromArgb(200, 16, 46);   // #C8102E
        private static readonly Color White = Color.White;

        /// <summary>Australian flag bitmap, width:height = 2:1. Super-sampled then scaled for crisp edges.</summary>
        public static Bitmap Australian(int width)
        {
            int height = Math.Max(2, width / 2);
            const int ss = 4;                          // super-sample factor
            using var big = new Bitmap(width * ss, height * ss);
            using (var g = Graphics.FromImage(big))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                Render(g, width * ss, height * ss);
            }
            var outp = new Bitmap(width, height);
            using (var g = Graphics.FromImage(outp))
            {
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.DrawImage(big, new Rectangle(0, 0, width, height));
            }
            return outp;
        }

        private static void Render(Graphics g, float W, float H)
        {
            using var blue = new SolidBrush(Blue);
            g.FillRectangle(blue, 0, 0, W, H);

            // --- Union Jack canton: upper hoist quarter (W/2 x H/2) ---
            float cw = W / 2f, ch = H / 2f;
            var canton = new RectangleF(0, 0, cw, ch);
            var clip = g.Clip;
            g.SetClip(canton);

            using (var whitePen = new Pen(White, ch / 3f))
            using (var redDiag = new Pen(Red, ch / 9f))
            {
                // St Andrew's saltire (white), then St Patrick's (red) on the diagonals.
                g.DrawLine(whitePen, 0, 0, cw, ch);
                g.DrawLine(whitePen, cw, 0, 0, ch);
                g.DrawLine(redDiag, 0, 0, cw, ch);
                g.DrawLine(redDiag, cw, 0, 0, ch);
            }
            // St George's cross: white bands then red bands, centred.
            using (var white = new SolidBrush(White))
            using (var red = new SolidBrush(Red))
            {
                float wv = cw * 0.34f, wh = ch * 0.34f;    // white band widths
                g.FillRectangle(white, cw / 2 - wv / 2, 0, wv, ch);
                g.FillRectangle(white, 0, ch / 2 - wh / 2, cw, wh);
                float rv = cw * 0.20f, rh = ch * 0.20f;    // red band widths
                g.FillRectangle(red, cw / 2 - rv / 2, 0, rv, ch);
                g.FillRectangle(red, 0, ch / 2 - rh / 2, cw, rh);
            }
            g.Clip = clip;

            // --- Stars (white): Commonwealth Star + Southern Cross ---
            using var star = new SolidBrush(White);
            // Commonwealth Star (7-pointed), centred under the canton.
            DrawStar(g, star, new PointF(W * 0.25f, H * 0.75f), H * 0.19f, 7);
            // Southern Cross on the fly (fractions of a width-2/height-1 flag, converted to W/H).
            DrawStar(g, star, new PointF(W * 0.75f, H * 0.84f), H * 0.10f, 7);   // Alpha
            DrawStar(g, star, new PointF(W * 0.625f, H * 0.50f), H * 0.10f, 7);  // Beta
            DrawStar(g, star, new PointF(W * 0.75f, H * 0.16f), H * 0.10f, 7);   // Gamma
            DrawStar(g, star, new PointF(W * 0.875f, H * 0.375f), H * 0.09f, 7); // Delta
            DrawStar(g, star, new PointF(W * 0.80f, H * 0.58f), H * 0.055f, 5);  // Epsilon
        }

        private static void DrawStar(Graphics g, Brush b, PointF c, float rOuter, int points)
        {
            float rInner = rOuter * (points == 5 ? 0.42f : 0.46f);
            using var path = new GraphicsPath();
            var pts = new PointF[points * 2];
            double start = -Math.PI / 2;               // a point faces up
            for (int i = 0; i < points * 2; i++)
            {
                double a = start + i * Math.PI / points;
                float r = (i % 2 == 0) ? rOuter : rInner;
                pts[i] = new PointF(c.X + (float)(r * Math.Cos(a)), c.Y + (float)(r * Math.Sin(a)));
            }
            path.AddPolygon(pts);
            g.FillPath(b, path);
        }
    }
}

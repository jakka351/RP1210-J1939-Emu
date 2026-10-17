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

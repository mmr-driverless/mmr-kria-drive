#ifndef CONTROLNODE_VIZ_MSGS_MSGHELPERS_HPP
#define CONTROLNODE_VIZ_MSGS_MSGHELPERS_HPP

#define __SET2(msg, Xname, X, Yname, Y) \
do { \
  msg.Xname = X; \
  msg.Yname = Y; \
} while (false)

#define __SET3(msg, Xname, X, Yname, Y, Zname, Z) \
do { \
  __SET2(msg, Xname, X, Yname, Y); \
  msg.Zname = Z; \
} while (false)

#define __SET4(msg, Xname, X, Yname, Y, Zname, Z, Wname, W) \
do { \
  __SET3(msg, Xname, X, Yname, Y, Zname, Z); \
  msg.Wname = W; \
} while (false)

#define SET_XY(msg, X, Y) __SET2(msg, x, X, y, Y)
#define SET_XYZ(msg, X, Y, Z) __SET3(msg, x, X, y, Y, z, Z)
#define SET_RGBA(msg, R, G, B, A) __SET4(msg, r, R, g, G, b, B, a, A)
#define SET_XYZW(msg, X, Y, Z, W) __SET4(msg, x, X, y, Y, z, Z, w, W)

#endif // !CONTROLNODE_VIZ_MSGS_MSGHELPERS_HPP
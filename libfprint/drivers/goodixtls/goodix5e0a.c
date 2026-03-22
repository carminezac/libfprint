// Goodix Tls driver for libfprint — 5e0a device
//
// Copyright (C) 2021 Alexander Meiler <alex.meiler@protonmail.com>
// Copyright (C) 2021 Matthieu CHARETTE <matthieu.charette@gmail.com>
// Copyright (C) 2021 Natasha England-Elbro <ashenglandelbro@protonmail.com>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#include "fp-device.h"
#include "fp-image-device.h"
#include "fp-image.h"
#include "fpi-context.h"
#include "fpi-image-device.h"
#include "fpi-image.h"
#include "fpi-ssm.h"

#define FP_COMPONENT "goodixtls5e0a"

#include <glib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "drivers_api.h"
#include "goodix.h"
#include "goodix_proto.h"
#include "goodix5e0a.h"

typedef unsigned short Goodix5e0aPix;

struct _FpiDeviceGoodixTls5e0a
{
  FpiDeviceGoodixTls parent;

  guint8 image_psk[32];
  gboolean has_image_psk;

  Goodix5e0aPix *calibration_img;  // baseline frame (no finger) for subtraction
};

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixTls5e0a, fpi_device_goodixtls5e0a, FPI,
                      DEVICE_GOODIXTLS5E0A, FpiDeviceGoodixTls);

G_DEFINE_TYPE (FpiDeviceGoodixTls5e0a, fpi_device_goodixtls5e0a,
               FPI_TYPE_DEVICE_GOODIXTLS);

// Forward declarations
static void goodix_5e0a_decode_frame (Goodix5e0aPix *frame, guint32 raw_size, const guint8 *raw_frame);
static void on_calibration_image (FpDevice *dev, guint8 *data, guint16 len, gpointer user_data, GError *err);

// ---- CALIBRATION ----

static void
on_calibration_image (FpDevice *dev, guint8 *data, guint16 len,
                      gpointer user_data, GError *err)
{
  FpiSsm *ssm = user_data;

  if (err)
    {
      fp_warn ("Calibration capture failed (non-fatal): %s", err->message);
      g_error_free (err);
      fpi_ssm_next_state (ssm);
      return;
    }

  FpiDeviceGoodixTls5e0a *self = FPI_DEVICE_GOODIXTLS5E0A (dev);

  if (len < GOODIX_5E0A_RAW_FRAME_SIZE)
    {
      fp_warn ("Calibration data too small: %d", len);
      fpi_ssm_next_state (ssm);
      return;
    }

  if (!self->calibration_img)
    self->calibration_img = calloc (GOODIX_5E0A_FRAME_SIZE,
                                    sizeof (Goodix5e0aPix));

  goodix_5e0a_decode_frame (self->calibration_img,
                            GOODIX_5E0A_RAW_FRAME_SIZE, data);
  fp_dbg ("Calibration frame captured (%d pixels)", GOODIX_5E0A_FRAME_SIZE);
  fpi_ssm_next_state (ssm);
}

static void
linear_subtract_5e0a (Goodix5e0aPix *src, const Goodix5e0aPix *baseline,
                      guint16 len)
{
  // Subtract baseline from scan: result = clamp(scan - baseline, 0, max)
  // This removes fixed pattern noise
  for (guint16 i = 0; i < len; i++)
    {
      if (src[i] > baseline[i])
        src[i] = src[i] - baseline[i];
      else
        src[i] = 0;
    }
}

// ---- HELPERS ----

static void
check_none_5e0a (FpDevice *dev, gpointer user_data, GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (user_data, error);
      return;
    }
  fpi_ssm_next_state (user_data);
}

static void
check_none_cmd_5e0a (FpDevice *dev, guint8 *data, guint16 len,
                     gpointer ssm, GError *err)
{
  if (err)
    {
      fpi_ssm_mark_failed (ssm, err);
      return;
    }
  fpi_ssm_next_state (ssm);
}

// ---- FRAME DECODE ----

// Decode the 4/6 byte packing used by Goodix sensors.
// The 5e0a raw frame has NO 8-byte header / 5-byte footer (unlike 5110).
static void
goodix_5e0a_decode_frame (Goodix5e0aPix *frame, guint32 raw_size,
                          const guint8 *raw_frame)
{
  Goodix5e0aPix *pix = frame;

  for (guint32 i = 0; i < raw_size; i += 6)
    {
      const guint8 *chunk = raw_frame + i;
      *pix++ = ((chunk[0] & 0xf) << 8) + chunk[1];
      *pix++ = (chunk[3] << 4) + (chunk[0] >> 4);
      *pix++ = ((chunk[5] & 0xf) << 8) + chunk[2];
      *pix++ = (chunk[4] << 4) + (chunk[5] >> 4);
    }
}

static void
goodix_5e0a_squash_frame (Goodix5e0aPix *frame, guint8 *squashed,
                          guint16 frame_size)
{
  // The 5e0a sensor has a circular active area with dead (zero) pixels
  // around the edges. Exclude zero pixels from min/max to preserve contrast.
  Goodix5e0aPix min = 0xffff;
  Goodix5e0aPix max = 0;

  for (int i = 0; i != frame_size; ++i)
    {
      if (frame[i] == 0)
        continue;  // skip dead zone pixels
      if (frame[i] < min)
        min = frame[i];
      if (frame[i] > max)
        max = frame[i];
    }

  // Fallback if all pixels are zero
  if (min > max)
    {
      min = 0;
      max = 1;
    }

  for (int i = 0; i != frame_size; ++i)
    {
      if (frame[i] == 0)
        {
          // Fill dead zone with white (background) so NBIS doesn't
          // confuse it with ridge pixels
          squashed[i] = 0xff;
          continue;
        }
      if (max == min)
        squashed[i] = 0xff;
      else
        squashed[i] = (frame[i] - min) * 0xff / (max - min);
    }
}

// ---- ACTIVATION STATE MACHINE ----

enum activate_5e0a_states {
  ACTIVATE_READ_AND_NOP1,
  ACTIVATE_ENABLE_CHIP,
  ACTIVATE_NOP2,
  ACTIVATE_CHECK_PSK,
  ACTIVATE_CMD_TLS,
  ACTIVATE_POV_IMAGE_CHECK,
  ACTIVATE_IMG_TLS,
  ACTIVATE_CALIBRATE,
  ACTIVATE_DONE,

  ACTIVATE_5E0A_NUM_STATES,
};

static void
on_psk_read_5e0a (FpDevice *dev, gboolean success, guint32 flags,
                  guint8 *psk, guint16 length, gpointer user_data,
                  GError *error)
{
  FpiSsm *ssm = user_data;

  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }

  if (!success)
    {
      fp_warn ("PSK read returned failure, continuing with placeholder PSK");
      fpi_ssm_next_state (ssm);
      return;
    }

  g_autofree gchar *psk_str = data_to_str (psk, length);
  fp_dbg ("Device PSK hash: 0x%s (flags: 0x%08x)", psk_str, flags);

  // The psk returned by preset_psk_read with flags 0xbb020001 is a hash/check,
  // not the actual image PSK. The real PSK must be obtained separately
  // (e.g., from Windows registry via DPAPI, or from a config file).

  fpi_ssm_next_state (ssm);
}

static void
on_cmd_tls_complete (FpDevice *dev, gpointer user_data, GError *error)
{
  FpiSsm *ssm = user_data;

  if (error)
    {
      fp_err ("Command TLS init failed: %s", error->message);
      fpi_ssm_mark_failed (ssm, error);
      return;
    }
  fp_dbg ("Command TLS established");
  fpi_ssm_next_state (ssm);
}

static void
on_pov_image_check_done (FpDevice *dev, guint8 *data, guint16 len,
                         gpointer ssm, GError *err)
{
  if (err)
    {
      fp_warn ("POV image check failed: %s — continuing anyway", err->message);
      g_error_free (err);
    }
  else
    {
      fp_dbg ("POV image check OK");
    }
  fpi_ssm_next_state (ssm);
}

static void
on_img_tls_complete (FpDevice *dev, gpointer user_data, GError *error)
{
  FpiSsm *ssm = user_data;

  if (error)
    {
      fp_err ("Image TLS init failed: %s", error->message);
      fpi_ssm_mark_failed (ssm, error);
      return;
    }
  fp_dbg ("Image TLS established");
  fpi_ssm_next_state (ssm);
}

static void
activate_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixTls5e0a *self = FPI_DEVICE_GOODIXTLS5E0A (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ACTIVATE_READ_AND_NOP1:
      goodix_start_read_loop (dev);
      goodix_send_nop (dev, check_none_5e0a, ssm);
      break;

    case ACTIVATE_ENABLE_CHIP:
      goodix_send_enable_chip (dev, TRUE, check_none_5e0a, ssm);
      break;

    case ACTIVATE_NOP2:
      goodix_send_nop (dev, check_none_5e0a, ssm);
      break;

    case ACTIVATE_CHECK_PSK:
      goodix_send_preset_psk_read (dev, GOODIX_5E0A_PSK_FLAGS, 0,
                                   on_psk_read_5e0a, ssm);
      break;

    case ACTIVATE_CMD_TLS:
      // Command TLS with PSK = 32 zero bytes (default)
      goodix_tls_init (dev, on_cmd_tls_complete, ssm);
      break;

    case ACTIVATE_POV_IMAGE_CHECK:
      goodix_send_pov_image_check (dev, on_pov_image_check_done, ssm);
      break;

    case ACTIVATE_IMG_TLS:
      {
        // Image TLS with the device-specific PSK.
        // TODO: Read the actual device-specific PSK from a config file
        // (e.g., ~/.config/libfprint/goodix-5e0a.psk) or extract it at runtime.
        // For now, use the PSK stored in self->image_psk if available,
        // otherwise fall back to 32 zero bytes as a placeholder.
        const guint8 *psk = self->image_psk;
        guint psk_len = 32;

        if (!self->has_image_psk)
          {
            fp_warn ("No device-specific image PSK available, using zeros. "
                     "Image decryption will fail unless the correct PSK is "
                     "provided.");
          }

        goodix_tls_init_image (dev, psk, psk_len, on_img_tls_complete, ssm);
      }
      break;

    case ACTIVATE_CALIBRATE:
      // Calibration disabled — skip to done
      fpi_ssm_next_state (ssm);
      break;

    case ACTIVATE_DONE:
      fpi_ssm_next_state (ssm);
      break;
    }
}

static void
activate_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  G_DEBUG_HERE ();
  if (error)
    {
      fp_err ("Failed during 5e0a activation: %s (code: %d)",
              error->message, error->code);
    }
  fpi_image_device_activate_complete (FP_IMAGE_DEVICE (dev), error);
}

// ---- SCAN STATE MACHINE ----

enum scan_5e0a_states {
  SCAN_QUERY_MCU,
  SCAN_FDT_DOWN,
  SCAN_GET_IMAGE,
  SCAN_DONE,

  SCAN_5E0A_NUM_STATES,
};

static void
scan_on_read_img_5e0a (FpDevice *dev, guint8 *data, guint16 len,
                       gpointer ssm, GError *err)
{
  if (err)
    {
      fpi_ssm_mark_failed (ssm, err);
      return;
    }

  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);

  fp_dbg ("Got decrypted image data: %d bytes", len);

  // The decrypted data is the raw frame (10560 bytes) followed by a
  // 4-byte footer/checksum. Use only the first RAW_FRAME_SIZE bytes.
  if (len < GOODIX_5E0A_RAW_FRAME_SIZE)
    fp_warn ("Image data too small: %d < %d", len, GOODIX_5E0A_RAW_FRAME_SIZE);

  Goodix5e0aPix *raw_frame = calloc (GOODIX_5E0A_FRAME_SIZE,
                                     sizeof (Goodix5e0aPix));
  goodix_5e0a_decode_frame (raw_frame, GOODIX_5E0A_RAW_FRAME_SIZE, data);

  // NOTE: Calibration subtraction disabled — it destroys signal for this sensor.
  // The 5e0a sensor doesn't benefit from baseline subtraction like the 511 does.

  guint8 *squashed = calloc (GOODIX_5E0A_FRAME_SIZE, 1);
  goodix_5e0a_squash_frame (raw_frame, squashed, GOODIX_5E0A_FRAME_SIZE);
  free (raw_frame);

  // Create the image — sensor produces 88-wide x 80-tall frames
  // (GOODIX_5E0A_HEIGHT is the PGM width, GOODIX_5E0A_WIDTH is the PGM height)
  FpImage *img = fp_image_new (GOODIX_5E0A_HEIGHT, GOODIX_5E0A_WIDTH);
  // No flags — let NBIS process the full image without any special handling
  // NBIS works best at 500 DPI = 19.685 ppmm
  img->ppmm = 19.685;
  memcpy (img->data, squashed, GOODIX_5E0A_FRAME_SIZE);
  free (squashed);

  // Debug: save raw decrypted data and processed image
  {
    FILE *fd;
    fd = fopen ("/tmp/goodix_5e0a_raw.bin", "wb");
    if (fd) { fwrite (data, 1, len, fd); fclose (fd); }

    fd = fopen ("/tmp/goodix_5e0a_image.pgm", "w");
    if (fd)
      {
        fprintf (fd, "P5 %d %d 255\n", GOODIX_5E0A_WIDTH, GOODIX_5E0A_HEIGHT);
        fwrite (img->data, 1, GOODIX_5E0A_FRAME_SIZE, fd);
        fclose (fd);
        fp_dbg ("Saved debug image to /tmp/goodix_5e0a_image.pgm");
      }
  }

  fpi_image_device_image_captured (img_dev, img);
  fpi_ssm_next_state (ssm);
}

static void
on_query_mcu_ack_5e0a (FpDevice *dev, gpointer user_data, GError *error)
{
  FpiSsm *ssm = user_data;

  if (error)
    {
      fp_warn ("query_mcu_state ACK error (non-fatal): %s", error->message);
      g_error_free (error);
    }
  fpi_ssm_next_state (ssm);
}

static void
scan_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SCAN_QUERY_MCU:
      {
        // 5e0a: send query_mcu_state with reply=FALSE (fire-and-forget).
        // The Python driver sends b"\x00\x01\x00" but the device only
        // needs the ACK, not a data reply.
        GoodixCallbackInfo *cb_info = malloc (sizeof (GoodixCallbackInfo));
        cb_info->callback = G_CALLBACK (on_query_mcu_ack_5e0a);
        cb_info->user_data = ssm;
        guint8 payload[] = { 0x55 };
        goodix_send_protocol (dev, GOODIX_CMD_QUERY_MCU_STATE, payload,
                              sizeof (payload), NULL, TRUE, GOODIX_TIMEOUT,
                              FALSE, goodix_receive_none, cb_info);
      }
      break;

    case SCAN_FDT_DOWN:
      fp_dbg ("Waiting for finger (FDT_DOWN)...");
      {
        // Send FDT_DOWN payload directly via goodix_send_protocol to avoid the
        // hardcoded 0x0c prefix that goodix_send_mcu_switch_to_fdt_down adds.
        // The 5e0a device uses 0x1c as the mode byte (already in the payload),
        // not 0x0c which is for other Goodix models.
        GoodixCallbackInfo *cb_info = malloc (sizeof (GoodixCallbackInfo));
        cb_info->callback = G_CALLBACK (check_none_cmd_5e0a);
        cb_info->user_data = ssm;
        goodix_send_protocol (dev, GOODIX_CMD_MCU_SWITCH_TO_FDT_DOWN,
                              goodix_5e0a_fdt_down_mode,
                              sizeof (goodix_5e0a_fdt_down_mode),
                              NULL, TRUE, 0, TRUE,
                              goodix_receive_default, cb_info);
      }
      break;

    case SCAN_GET_IMAGE:
      fpi_image_device_report_finger_status (img_dev, TRUE);
      goodix_tls_read_image_5e0a_with_payload (
        dev,
        goodix_5e0a_get_image_payload,
        sizeof (goodix_5e0a_get_image_payload),
        scan_on_read_img_5e0a, ssm);
      break;

    case SCAN_DONE:
      fpi_image_device_report_finger_status (img_dev, FALSE);
      fpi_ssm_next_state (ssm);
      break;
    }
}

static void
scan_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  if (error)
    {
      fp_err ("Failed to scan: %s (code: %d)", error->message, error->code);
      fpi_image_device_session_error (FP_IMAGE_DEVICE (dev), error);
      return;
    }
  fp_dbg ("5e0a scan finished");
}

// ---- DEVICE CALLBACKS ----

static void
dev_activate (FpImageDevice *img_dev)
{
  FpDevice *dev = FP_DEVICE (img_dev);

  fpi_ssm_start (fpi_ssm_new (dev, activate_run_state,
                               ACTIVATE_5E0A_NUM_STATES),
                 activate_complete);
}

static void
dev_change_state (FpImageDevice *img_dev, FpiImageDeviceState state)
{
  if (state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON)
    {
      fpi_ssm_start (fpi_ssm_new (FP_DEVICE (img_dev), scan_run_state,
                                   SCAN_5E0A_NUM_STATES),
                     scan_complete);
    }
}

static void
dev_deactivate (FpImageDevice *img_dev)
{
  FpDevice *dev = FP_DEVICE (img_dev);

  goodix_reset_state (dev);

  FpiDeviceGoodixTls5e0a *self = FPI_DEVICE_GOODIXTLS5E0A (dev);
  g_clear_pointer (&self->calibration_img, free);

  GError *error = NULL;
  goodix_shutdown_tls (dev, &error);
  goodix_shutdown_image_tls (dev, &error);

  fpi_image_device_deactivate_complete (img_dev, error);
}

static void
dev_init (FpImageDevice *img_dev)
{
  FpDevice *dev = FP_DEVICE (img_dev);
  GError *error = NULL;

  if (goodix_dev_init (dev, &error))
    {
      fpi_image_device_open_complete (img_dev, error);
      return;
    }

  fpi_image_device_open_complete (img_dev, NULL);
}

static void
dev_deinit (FpImageDevice *img_dev)
{
  FpDevice *dev = FP_DEVICE (img_dev);
  GError *error = NULL;

  if (goodix_dev_deinit (dev, &error))
    {
      fpi_image_device_close_complete (img_dev, error);
      return;
    }

  fpi_image_device_close_complete (img_dev, NULL);
}

// ---- TYPE INIT ----

static gboolean
load_psk_from_file (FpiDeviceGoodixTls5e0a *self)
{
  // Try multiple config file locations
  const char *paths[] = {
    "/etc/libfprint/goodix-5e0a.psk",
    NULL,   // filled with $HOME path below
  };

  // Build home-based path
  const char *home = g_get_home_dir ();
  g_autofree char *home_path = NULL;
  if (home)
    {
      home_path = g_strdup_printf ("%s/.config/libfprint/goodix-5e0a.psk", home);
      paths[1] = home_path;
    }

  for (int i = 0; i < 2; i++)
    {
      if (!paths[i])
        continue;

      g_autofree char *contents = NULL;
      gsize len = 0;

      if (!g_file_get_contents (paths[i], &contents, &len, NULL))
        continue;

      // Strip whitespace/newlines
      g_strstrip (contents);
      gsize hex_len = strlen (contents);

      if (hex_len < 64)
        {
          fp_warn ("PSK file %s too short (%zu chars, need 64 hex)", paths[i], hex_len);
          continue;
        }

      // Parse hex string to bytes
      gboolean valid = TRUE;
      for (int j = 0; j < 32 && valid; j++)
        {
          char byte_str[3] = { contents[j*2], contents[j*2+1], 0 };
          char *endp;
          unsigned long val = strtoul (byte_str, &endp, 16);
          if (*endp != 0)
            {
              valid = FALSE;
              break;
            }
          self->image_psk[j] = (guint8) val;
        }

      if (valid)
        {
          fp_info ("Loaded image PSK from %s", paths[i]);
          self->has_image_psk = TRUE;
          return TRUE;
        }
      else
        {
          fp_warn ("Invalid hex in PSK file %s", paths[i]);
        }
    }

  return FALSE;
}

static void
fpi_device_goodixtls5e0a_init (FpiDeviceGoodixTls5e0a *self)
{
  memset (self->image_psk, 0, sizeof (self->image_psk));
  self->has_image_psk = FALSE;

  // Load device-specific PSK from config file.
  // The PSK is unique per device, extracted via extract_psk.py.
  // Store it as 64 hex chars in one of:
  //   /etc/libfprint/goodix-5e0a.psk
  //   ~/.config/libfprint/goodix-5e0a.psk
  if (!load_psk_from_file (self))
    fp_warn ("No image PSK loaded — image TLS will fail. "
             "Run extract_psk.py and save PSK to "
             "/etc/libfprint/goodix-5e0a.psk");
}

static void
fpi_device_goodixtls5e0a_class_init (FpiDeviceGoodixTls5e0aClass *class)
{
  FpiDeviceGoodixTlsClass *gx_class = FPI_DEVICE_GOODIXTLS_CLASS (class);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (class);
  FpImageDeviceClass *img_dev_class = FP_IMAGE_DEVICE_CLASS (class);

  gx_class->interface = GOODIX_5E0A_INTERFACE;
  gx_class->ep_in = GOODIX_5E0A_EP_IN;
  gx_class->ep_out = GOODIX_5E0A_EP_OUT;

  dev_class->id = "goodixtls5e0a";
  dev_class->full_name = "Goodix TLS Fingerprint Sensor 5e0a";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = goodix_5e0a_id_table;
  dev_class->nr_enroll_stages = 10;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  img_dev_class->bz3_threshold = 5;
  // Sensor frame layout: 88 pixels wide, 80 pixels tall
  img_dev_class->img_width = GOODIX_5E0A_HEIGHT;   // 88
  img_dev_class->img_height = GOODIX_5E0A_WIDTH;   // 80

  img_dev_class->activate = dev_activate;
  img_dev_class->change_state = dev_change_state;
  img_dev_class->deactivate = dev_deactivate;
  img_dev_class->img_open = dev_init;
  img_dev_class->img_close = dev_deinit;

  fpi_device_class_auto_initialize_features (dev_class);
}

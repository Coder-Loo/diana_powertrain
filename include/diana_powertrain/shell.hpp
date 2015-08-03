#pragma once

enum OSCommandStatus {
      COMPLETED_NO_REPLY = 0,
      COMPLETED_REPLY = 1,
      REJECTED_REPLY = 3,
      EXECUTING = 255
};

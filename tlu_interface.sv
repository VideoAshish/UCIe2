// ============================================================
// TileLink Uncached Interface (TL-UH)
// Reference: OpenTitan tl_interface.sv[reference:20][reference:21]
// ============================================================

interface tlu_interface #(
    parameter TLAddressWidth = 32,
    parameter TLDataWidth   = 64,
    parameter TLIDWidth     = 8
);

    localparam MaskWidth = TLDataWidth / 8;

    // Type definitions[reference:22]
    typedef logic [TLIDWidth-1:0]     id_t;
    typedef logic [TLAddressWidth-1:0] address_t;
    typedef logic [TLDataWidth-1:0]   data_t;
    typedef logic [MaskWidth-1:0]     mask_t;

    // A-Channel control (Request)[reference:23]
    typedef struct packed {
        a_opcode_e opcode;
        param_t    param;
        size_t     size;
        id_t       source;
        address_t  address;
    } a_control_t;

    typedef struct packed {
        mask_t     mask;
        data_t     data;
        logic      corrupt;
    } a_data_t;

    // D-Channel control (Response)[reference:24]
    typedef struct packed {
        d_opcode_e opcode;
        param_t    param;
        size_t     size;
        id_t       source;
        id_t       sink;
        logic      denied;
    } d_control_t;

    typedef struct packed {
        data_t     data;
        logic      corrupt;
    } d_data_t;

    // Channel A: Request (Initiator → Target)
    a_control_t ac;
    a_data_t    ad;
    logic       av;  // valid
    logic       ar;  // ready

    // Channel D: Response (Target → Initiator)
    d_control_t dc;
    d_data_t    dd;
    logic       dv;  // valid
    logic       dr;  // ready

    // Modport definitions[reference:25]
    modport initiator(
        output ac, ad, av,
        input  ar,
        input  dc, dd, dv,
        output dr
    );

    modport target(
        input  ac, ad, av,
        output ar,
        output dc, dd, dv,
        input  dr
    );

endinterface
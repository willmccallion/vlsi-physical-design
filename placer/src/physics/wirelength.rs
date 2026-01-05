use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;

pub fn compute_wa_gradient(
    db: &NetlistDB,
    positions: &[Point<f64>],
    gamma: f64,
    gradients: &mut [Point<f64>],
) -> f64 {
    let mut total_wl = 0.0;
    let inv_gamma = 1.0 / gamma;

    for net in &db.nets {
        if net.pins.len() < 2 {
            continue;
        }

        let mut max_x = f64::NEG_INFINITY;
        let mut min_x = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;

        // Find bounds for LogSumExp stability
        for &pin_id in &net.pins {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let pos = db.get_pin_position(pin_id, &positions[cell_id.index()]);
            max_x = max_x.max(pos.x);
            min_x = min_x.min(pos.x);
            max_y = max_y.max(pos.y);
            min_y = min_y.min(pos.y);
        }

        // Compute WA terms
        let mut sum_exp_x_pos = 0.0;
        let mut sum_x_exp_x_pos = 0.0;
        let mut sum_exp_x_neg = 0.0;
        let mut sum_x_exp_x_neg = 0.0;

        let mut sum_exp_y_pos = 0.0;
        let mut sum_y_exp_y_pos = 0.0;
        let mut sum_exp_y_neg = 0.0;
        let mut sum_y_exp_y_neg = 0.0;

        for &pin_id in &net.pins {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let pos = db.get_pin_position(pin_id, &positions[cell_id.index()]);

            let exp_x_pos = ((pos.x - max_x) * inv_gamma).exp();
            let exp_x_neg = ((min_x - pos.x) * inv_gamma).exp();
            let exp_y_pos = ((pos.y - max_y) * inv_gamma).exp();
            let exp_y_neg = ((min_y - pos.y) * inv_gamma).exp();

            sum_exp_x_pos += exp_x_pos;
            sum_x_exp_x_pos += pos.x * exp_x_pos;

            sum_exp_x_neg += exp_x_neg;
            sum_x_exp_x_neg += pos.x * exp_x_neg;

            sum_exp_y_pos += exp_y_pos;
            sum_y_exp_y_pos += pos.y * exp_y_pos;

            sum_exp_y_neg += exp_y_neg;
            sum_y_exp_y_neg += pos.y * exp_y_neg;
        }

        let wa_x_pos = sum_x_exp_x_pos / sum_exp_x_pos;
        let wa_x_neg = sum_x_exp_x_neg / sum_exp_x_neg;
        let wa_y_pos = sum_y_exp_y_pos / sum_exp_y_pos;
        let wa_y_neg = sum_y_exp_y_neg / sum_exp_y_neg;

        total_wl += (wa_x_pos - wa_x_neg) + (wa_y_pos - wa_y_neg);

        // Compute gradients
        // dWA/dx_i = ( (1 + (x_i - WA)/gamma) * exp(...) ) / Sum exp
        for &pin_id in &net.pins {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let pos = db.get_pin_position(pin_id, &positions[cell_id.index()]);

            let exp_x_pos = ((pos.x - max_x) * inv_gamma).exp();
            let exp_x_neg = ((min_x - pos.x) * inv_gamma).exp();
            let exp_y_pos = ((pos.y - max_y) * inv_gamma).exp();
            let exp_y_neg = ((min_y - pos.y) * inv_gamma).exp();

            let grad_x = ((1.0 + (pos.x - wa_x_pos) * inv_gamma) * exp_x_pos / sum_exp_x_pos)
                - ((1.0 - (pos.x - wa_x_neg) * inv_gamma) * exp_x_neg / sum_exp_x_neg);

            let grad_y = ((1.0 + (pos.y - wa_y_pos) * inv_gamma) * exp_y_pos / sum_exp_y_pos)
                - ((1.0 - (pos.y - wa_y_neg) * inv_gamma) * exp_y_neg / sum_exp_y_neg);

            gradients[cell_id.index()].x += grad_x * net.weight;
            gradients[cell_id.index()].y += grad_y * net.weight;
        }
    }
    total_wl
}
